/*
 * palinput.h
 *
 *  Created on: 25 июля 2020 г.
 *      Author: andru
 */

#ifndef PALINPUT_H_
#define PALINPUT_H_

#define LINE_DELAY			15		// line read interval, ms
#define LINE_ON				0x00	// value for line on
#define LINE_OFF			0xFF	// value for line off
#define LINE_NUM			4		// number of lines

// line event masks
#define EVT_INPUT_IN1		(1 << 0)
#define EVT_INPUT_IN2		(1 << 1)
#define EVT_INPUT_IN3		(1 << 2)
#define EVT_INPUT_IN4		(1 << 3)

typedef void (*line_callback_t)(void *arg);
typedef struct _io_line_event_t io_line_event_t;
struct _io_line_event_t {
	ioline_t line;
	line_callback_t cb;
};

#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint8_t input_line[LINE_NUM];
void input_init(void);

#ifdef __cplusplus
}
#endif

#endif /* PALINPUT_H_ */
