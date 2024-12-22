/*
 * lcd.h
 *
 * Created: 2024-12-14 오전 3:17:41
 *  Author: ghdtj
 */ 


/****************** lcd.h ********************************************/

#include <avr/io.h>

#define  F_CPU 14.7456E6
#include <util/delay.h>

#define LCD_WDATA PORTA     // LCD 데이터 포트 정의
#define LCD_WINST PORTA
#define LCD_CTRL PORTG      // LCD 제어 신호 정의
#define LCD_EN 0            // Enable 신호
#define LCD_RW 1            // 데이터(1)/명령어(0)
#define LCD_RS 2            // 읽기(1)/쓰기(0)
#define On 1                // 불리언 상수 정의
#define Off 0
#define RIGHT 1
#define LEFT 0

typedef unsigned char Byte;

void LCD_Data(Byte ch);
void LCD_Comm(Byte ch);
void LCD_delay(Byte ms);
void LCD_CHAR(Byte c);
void LCD_Str(Byte *str);
void LCD_Pos(unsigned char col, unsigned char row);
void LCD_Clear(void);
void PortInit(void);
void LCD_Init(void);
void LCD_Shift(char p);
void Cursor_Home(void);
void Cursor_shift(Byte p);

void PortInit(void)
{
	DDRA = 0xFF; // PORTA를 출력으로
	DDRG = 0x0F; // PORTG의 하위 4비트를 출력으로
}

void LCD_Data(Byte ch)
{
	LCD_CTRL |= (1 << LCD_RS); // RS=1, R/W=0으로 데이터 쓰기 사이클
	LCD_CTRL &= ~(1 << LCD_RW);
	LCD_CTRL |= (1 << LCD_EN); // LCD Enable
	_delay_us(50); // 시간 지연
	LCD_WDATA = ch; // 데이터 출력
	_delay_us(50); // 시간 지연
	LCD_CTRL &= ~(1 << LCD_EN); // LCD Disable
}

void LCD_Comm(Byte ch)
{
	LCD_CTRL &= ~(1 << LCD_RS); // RS=R/W=0으로 명령어 쓰기 사이클
	LCD_CTRL &= ~(1 << LCD_RW);
	LCD_CTRL |= (1 << LCD_EN); // LCD Enable
	_delay_us(50); // 시간 지연
	LCD_WINST = ch; // 명령어 쓰기
	_delay_us(50); // 시간 지연
	LCD_CTRL &= ~(1 << LCD_EN); // LCD Disable
}

void LCD_CHAR(Byte c) // 한 문자 출력
{
	LCD_Data(c);
	_delay_ms(1);
}

void LCD_Str(Byte *str) // 문자열 출력
{
	while(*str != 0) {
		LCD_CHAR(*str);
		str++;
	}
}

void LCD_Pos(unsigned char col, unsigned char row) // LCD 포지션 설정
{
	LCD_Comm(0x80|(0x40*col + row)); // row = 문자행, col = 문자열
}

void Cursor_Home(void)
{
	LCD_Comm(0x02); // Cursor Home
	_delay_ms(2); // 2ms 지연
}

void LCD_Clear(void) // 화면 클리어 (1)
{
	LCD_Comm(0x01);
	_delay_ms(2);
}

void LCD_Shift(char p) // 디스플레이 시프트 (5)
{
	// 표시 화면 전체를 오른쪽으로 이동
	if(p == RIGHT) {
		LCD_Comm(0x1C);
		_delay_ms(1); // 시간 지연
	}
	// 표시 화면 전체를 왼쪽으로 이동
	else if(p == LEFT) {
		LCD_Comm(0x18);
		_delay_ms(1);
	}
}

void Cursor_shift(Byte p)
{
	if(p == RIGHT) {
		LCD_Comm(0x14);
		_delay_ms(1);
	}
	else if(p == LEFT) {
		LCD_Comm(0x10);
		_delay_ms(1);
	}
}

void LCD_Init(void) 	// LCD 초기화
{
	PortInit();
	
	LCD_Comm(0x30);	// 초기화 Set,
	_delay_us(4100);    // 4.1ms 지연
	LCD_Comm(0x30);   	// 초기화 Set,
	_delay_us(100); 	// 100us 지연
	LCD_Comm(0x30); 	// 초기화 Set,
	_delay_us(100); 	// 100us 지연
	LCD_Comm(0x38); 	// 초기화 Set, 데이터 길이 8Bit, 표시라인 2행 사용을 위한 기능 설정
	_delay_us(1000); 	// 명령을 처리 하는데 최소 40us 지연이 발생하기에 여유를 고려하여 설정
	LCD_Comm(0x0e); 	// Display ON, Cursor On, Blink Off
	_delay_us(1000); 	// 40㎲ 이상을 기다림
	LCD_Comm(0x01); 	// LCD Clear
	_delay_us(2000); 	// 1.64㎳ 이상을 기다림
	LCD_Comm(0x06); 	// Cursor Entry Mode Set, 표시 위치 +1씩 증가
	_delay_us(1000); 	// 40㎲ 이상을 기다림
}