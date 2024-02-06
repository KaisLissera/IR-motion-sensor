/*
 * ezhLib.h
 *
 *  Created on: 2023.07.31
 *      Author: Kais_Lissera
 */

#ifndef INC_LIB_H_
#define INC_LIB_H_

#include <board.h>

#include <stm32f072xb.h>

//Return values
/////////////////////////////////////////////////////////////////////

typedef enum {
	retvOk,
	retvFail,
	retvTimeout,
	retvBusy,
	retvInProgress,
	retvCmdError,
	retvCmdUnknown,
	retvBadValue,
	retvNew,
	retvSame,
	retvLast,
	retvEmpty,
	retvOverflow,
	retvNotANumber,
	retvWriteProtect,
	retvWriteError,
	retvEndOfFile,
	retvNotFound,
	retvBadState,
	retvDisconnected,
	retvCollision,
	retvCRCError,
	retvNACK,
	retvNoAnswer,
	retvOutOfMemory,
	retvNotAuthorised,
	retvNoChanges,
	retvTooHighSystemClock
} retv_t;

typedef enum {
	Enable 		= 0,
	Disable 	= 1
} Ability_t;

//Simple assert
/////////////////////////////////////////////////////////////////////

#define ASSERT_SIMPLE(x) if((x) == 0) { while(1); }

//Simple circular buffer template - not used for now
/////////////////////////////////////////////////////////////////////
//Example
//static CircBuffer_t<uint8_t,1024> test;

template <typename T, uint32_t Size>
class CircBuffer_t {
private:
	T Buffer_[Size];
	uint32_t start_;
	uint32_t end_;
public:
	CircBuffer_t() {
		start_ = 0;
		end_ = 0;
	}
	T Read() {
		T temp = Buffer_[start_];
		start_ = (start_ + 1) % Size;
		return temp;
	}
	uint32_t Length() {
		if (end_ > start_)
			return end_ - start_;
		else
			return Size - start_ + end_;
	}
	void Write(T data) {
		end_ = (end_ + 1) % Size;
		Buffer_[end_] = data;
	}
	void Clear() {
		start_ = 0;
		end_ = 0;
	}
};

//Commands templates
/////////////////////////////////////////////////////////////////////

/*Callback without arguments
 * Example:
 * void HelpCallback();
 * Command_t Help((char*)"help", HelpCallback);
*/
struct Command_t {
	const char* Name;
	void (*Callback)();

	Command_t(const char* _Name, void (*_Callback)()) {
		Name = _Name;
		Callback = _Callback;
	}
};

/*Callback with one argument
 * Example:
 * void EchoCallback(uint32_t);
 * CommandWithArgs_t Echo((char*)"echo", EchoCallback);
 */
struct CommandWithArgs_t {
	const char* Name;
	void (*Callback)(uint32_t);

	CommandWithArgs_t(const char* _Name, void (*_Callback)(uint32_t)) {
		Name = _Name;
		Callback = _Callback;
	}
};

#endif /* INC_LIB_H_ */
