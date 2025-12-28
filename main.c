#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <sys/select.h>
#include <sys/timerfd.h>
#include <termios.h>

#include "fifo8.h"
#include "ecp_defs.h"
#include "slip.h"
#include "crc8-ccitt.h"

int tim_fd = -1;
int ecp_fd = -1;

uint32_t ticks_625us = 0;
uint32_t ecp_last_rx_time = 0;
uint32_t ecp_last_tx_time = 0;
uint32_t ecp_local_rx_bytes = 0;
uint32_t ecp_local_tx_bytes = 0;
uint32_t ecp_peer_rx_bytes = 0;
uint32_t ecp_peer_tx_bytes = 0;
uint32_t rf_peer_rx_bytes = 0;
uint32_t rf_peer_tx_bytes = 0;
bool rf_peer_started = false;
bool rf_peer_running = false;
uint8_t rf_peer_channel = 0;
uint32_t rf_peer_access_addr = 0x0;
uint32_t rf_peer_crcinit = 0x0;

struct fifo8 fifo8_ecp_tx = FIFO8_INIT(4096);

#define ECP_BUFSIZE 768

uint8_t ecp_rxbuf[ECP_BUFSIZE];
uint16_t ecp_rxidx = 0;
uint8_t ecp_prev_char = 0;

uint64_t ecp_crcerr = 0;

uint8_t ping_data[256];

void ecp_handle_ping(void)
{
	if (memcmp(&ecp_rxbuf[6], ping_data, sizeof(ping_data)) == 0) {
		return;
	}
	printf("PING ERROR\r\n");
}

void ecp_handle_stat(void)
{
	memcpy(&ecp_peer_rx_bytes, &ecp_rxbuf[6], 4);
	memcpy(&ecp_peer_tx_bytes, &ecp_rxbuf[10], 4);
	memcpy(&rf_peer_rx_bytes, &ecp_rxbuf[14], 4);
	memcpy(&rf_peer_tx_bytes, &ecp_rxbuf[18], 4);
	rf_peer_started = !!(ecp_rxbuf[22] & (1 << 0));
	rf_peer_running = !!(ecp_rxbuf[22] & (1 << 1));
	rf_peer_channel = ecp_rxbuf[23];
	memcpy(&rf_peer_access_addr, &ecp_rxbuf[24], 4);
	memcpy(&rf_peer_crcinit, &ecp_rxbuf[28], 4);
}

void ecp_handle_rxpkt(void)
{
	uint16_t crcidx;
	crcidx = ecp_rxidx - 1;
	uint8_t crc = 0;
	unsigned int i;
	ecp_last_rx_time = ticks_625us;
	for (i = 0; i < crcidx; i++) {
		crc = crc8_ccitt_byte(crc, ecp_rxbuf[i]);
	}
	if (crc != ecp_rxbuf[crcidx]) {
		ecp_crcerr += 1;
		return;
	}

	switch (ecp_rxbuf[0]) {
	case ECP_CMD_PING:
		ecp_handle_ping();
		return;
	case ECP_CMD_STAT:
		ecp_handle_stat();
		return;
	}
}

void ecp_handle_char(uint8_t c)
{
	ecp_local_rx_bytes += 1;
	if (ecp_rxidx >= sizeof(ecp_rxbuf)) {
		ecp_rxidx = 0;
	}
	if (ecp_prev_char == SLIP_ESC) {
		switch (c) {
		case SLIP_ESC_END:
			ecp_rxbuf[ecp_rxidx] = SLIP_END;
			ecp_rxidx += 1;
			break;
		case SLIP_ESC_ESC:
			ecp_rxbuf[ecp_rxidx] = SLIP_ESC;
			ecp_rxidx += 1;
			break;
		}
		ecp_prev_char = c;
		return;
	}
	switch (c) {
	case SLIP_END:
		if (ecp_rxidx != 0) {
			ecp_handle_rxpkt();
		}
		ecp_rxidx = 0;
		break;
	case SLIP_ESC:
		break;
	default:
		ecp_rxbuf[ecp_rxidx] = c;
		ecp_rxidx += 1;
		break;
	}
	ecp_prev_char = c;
}

void ecp_read(void)
{
	int ret;
	uint8_t buf[BUFSIZ];
	ret = read(ecp_fd, buf, sizeof(buf));
	if (ret == 0) {
		fprintf(stderr, "ecp eof\r\n");
		exit(EXIT_FAILURE);
	}
	if (ret < 0) {
		perror("ecp read failed");
		exit(EXIT_FAILURE);
	}
	int i;
	for (i = 0; i < ret; i++) {
		ecp_handle_char(buf[i]);
	}
}

void ecp_write(void)
{
	int ret;
	uint8_t buf[64];
	uint32_t n;
	n = fifo8_num_used(&fifo8_ecp_tx);
	uint32_t i;
	for (i = 0; (i < n) && (i < 64); i++) {
		buf[i] = fifo8_pop(&fifo8_ecp_tx);
	}
	ret = write(ecp_fd, buf, i);
	if (ret < 0) {
		perror("ecp write failed");
		exit(EXIT_FAILURE);
	}
	if (ret < (signed)i) {
		fprintf(stderr, "ecp short write %d < %d\r\n", ret, i);
		exit(EXIT_FAILURE);
	}
	ecp_last_tx_time = ticks_625us;
	ecp_local_tx_bytes += ret;
}

void tim_read(void)
{
	uint64_t expr;
	int ret;
	ret = read(tim_fd, &expr, sizeof(expr));
	if (ret <= 0) {
		perror("tim read failed");
		exit(EXIT_FAILURE);
	}
	ticks_625us += expr;
}

void ecp_end(void)
{
	if (fifo8_num_free(&fifo8_ecp_tx) < 1) {
		return;
	}
	fifo8_push(&fifo8_ecp_tx, SLIP_END);
}

void ecp_send(uint8_t c)
{
	if (fifo8_num_free(&fifo8_ecp_tx) < 3) {
		return;
	}
	switch (c) {
	case SLIP_END:
		fifo8_push(&fifo8_ecp_tx, SLIP_ESC);
		fifo8_push(&fifo8_ecp_tx, SLIP_ESC_END);
		break;
	case SLIP_ESC:
		fifo8_push(&fifo8_ecp_tx, SLIP_ESC);
		fifo8_push(&fifo8_ecp_tx, SLIP_ESC_ESC);
		break;
	default:
		fifo8_push(&fifo8_ecp_tx, c);
		break;
	}
}

void ecp_send_buf(uint8_t *buf, unsigned int len)
{
	while (len > 0) {
		ecp_send(*buf);
		buf += 1;
		len -= 1;
	}
}

uint8_t ecp_seqn = 0;

void ecp_send_ping(uint8_t *p, uint16_t plen)
{
	uint32_t ticks;
	ticks = ticks_625us;
	uint8_t c = 0;
	uint8_t crc = 0;
	ecp_end();
	c = ECP_CMD_PING;
	ecp_send(c);
	crc = crc8_ccitt_byte(crc, c);
	c = ecp_seqn;
	ecp_send(c);
	crc = crc8_ccitt_byte(crc, c);
	ecp_seqn += 1;
	ecp_send_buf((uint8_t *)&ticks, sizeof(ticks));
	crc = crc8_ccitt_buf(crc, (uint8_t *)&ticks, sizeof(ticks));
	ecp_send_buf((uint8_t *)p, plen);
	crc = crc8_ccitt_buf(crc, (uint8_t *)p, plen);
	ecp_send(crc);
	ecp_end();
}

void ecp_send_stat(void)
{
	uint8_t c = 0;
	uint8_t crc = 0;
	ecp_end();
	c = ECP_CMD_STAT;
	ecp_send(c);
	crc = crc8_ccitt_byte(crc, c);
	c = ecp_seqn;
	ecp_send(c);
	crc = crc8_ccitt_byte(crc, c);
	ecp_seqn += 1;
	ecp_send(crc);
	ecp_end();
}

void ecp_send_rf_start(void)
{
	uint8_t c = 0;
	uint8_t crc = 0;
	ecp_end();
	c = ECP_CMD_RF_START;
	ecp_send(c);
	crc = crc8_ccitt_byte(crc, c);
	c = ecp_seqn;
	ecp_send(c);
	crc = crc8_ccitt_byte(crc, c);
	ecp_seqn += 1;
	ecp_send(crc);
	ecp_end();
}

void ecp_show_link_stat(void)
{
	static uint32_t prev_ticks = 0;
	if ((ticks_625us - prev_ticks) < 1600) {
		return;
	}
	//printf("ECP LAST D2H TIME: %d\r\n", ecp_last_rx_time);
	//printf("ECP LAST H2D TIME: %d\r\n", ecp_last_tx_time);
	static uint32_t prev_ecp_peer_rx_bytes = 0;
	static uint32_t prev_ecp_peer_tx_bytes = 0;
	printf("ECP H2D BPS: %d\r\n",
	       ecp_peer_rx_bytes - prev_ecp_peer_rx_bytes);
	prev_ecp_peer_rx_bytes = ecp_peer_rx_bytes;
	printf("ECP D2H BPS: %d\r\n",
	       ecp_peer_tx_bytes - prev_ecp_peer_tx_bytes);
	prev_ecp_peer_tx_bytes = ecp_peer_tx_bytes;

	static uint32_t prev_rf_peer_rx_bytes = 0;
	static uint32_t prev_rf_peer_tx_bytes = 0;
	printf("RF RX BPS: %d\r\n",
	       rf_peer_rx_bytes - prev_rf_peer_rx_bytes);
	prev_rf_peer_rx_bytes = rf_peer_rx_bytes;
	printf("RF TX BPS: %d\r\n",
	       rf_peer_tx_bytes - prev_rf_peer_tx_bytes);
	prev_rf_peer_tx_bytes = rf_peer_tx_bytes;
	printf("RF STARTED: %d\r\n", rf_peer_started);
	printf("RF RUNNING: %d\r\n", rf_peer_running);
	printf("RF CHANNEL: %d\r\n", rf_peer_channel);
	printf("RF ACCESS ADDRESS: %08X\r\n", rf_peer_access_addr);
	printf("RF CRCINIT: %06X\r\n", rf_peer_crcinit);
}

void periodic_1s_task(void)
{
	static uint32_t prev_ticks = 0;
	if ((ticks_625us - prev_ticks) < 1600) {
		return;
	}
	ecp_show_link_stat();
	ecp_send_ping(ping_data, sizeof(ping_data));
	ecp_send_stat();
	prev_ticks = ticks_625us;
}

void main_loop(void)
{
	fd_set rfds;
	fd_set wfds;
	struct timeval tv;
	int ret;
	while (1) {
		periodic_1s_task();
		FD_ZERO(&rfds);
		FD_SET(ecp_fd, &rfds);
		FD_SET(tim_fd, &rfds);
		FD_ZERO(&wfds);
		if (fifo8_num_used(&fifo8_ecp_tx) > 0) {
			FD_SET(ecp_fd, &wfds);
		}
		tv.tv_sec = 0;
		tv.tv_usec = 100;
		ret = select(1024, &rfds, &wfds, NULL, &tv);
		if (ret == 0) {
			continue;
		}
		if (ret < 0) {
			perror("select failed");
			exit(EXIT_FAILURE);
		}
		if (FD_ISSET(tim_fd, &rfds)) {
			tim_read();
		}
		if (FD_ISSET(ecp_fd, &rfds)) {
			ecp_read();
		}
		if (FD_ISSET(ecp_fd, &wfds)) {
			ecp_write();
		}
	}
}

int main(int argc, char *argv[])
{
	if (argc < 2) {
		fprintf(stderr, "usage: %s /dev/ttyACMX\r\n", argv[0]);
		exit(EXIT_FAILURE);
	}
	int ret;
	ecp_fd = open(argv[1], O_RDWR | O_CLOEXEC | O_NOCTTY);
	if (ecp_fd < 0) {
		perror("ecp open failed");
		exit(EXIT_FAILURE);
	}
	struct termios com_attr;
	ret = tcgetattr(ecp_fd, &com_attr);
	if (ret < 0) {
		perror("ecp get termios failed");
		exit(EXIT_FAILURE);
	}
	cfmakeraw(&com_attr);
	ret = tcsetattr(ecp_fd, TCSANOW, &com_attr);
	if (ret < 0) {
		perror("ecp set termios failed");
		exit(EXIT_FAILURE);
	}
	tim_fd = timerfd_create(CLOCK_REALTIME, 0);
	if (tim_fd < 0) {
		perror("timerfd create failed");
		exit(EXIT_FAILURE);
	}
	struct itimerspec timspec;
	bzero(&timspec, sizeof(timspec));
	timspec.it_interval.tv_sec = 0;
	timspec.it_interval.tv_nsec = 1 * 625000; // 625us
	timspec.it_value.tv_sec = 0;
	timspec.it_value.tv_nsec = 1;
	ret = timerfd_settime(tim_fd, 0, &timspec, NULL);
	if (ret < 0) {
		perror("timerfd settime failed");
		exit(EXIT_FAILURE);
	}
	unsigned int i;
	for (i = 0; i < sizeof(ping_data); i++) {
		ping_data[i] = i;
	}

	ecp_send_rf_start();
	main_loop();
	exit(EXIT_SUCCESS);
}
