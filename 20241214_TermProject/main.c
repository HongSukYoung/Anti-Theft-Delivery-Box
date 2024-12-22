/*
 * 20241214_TermProject.c
 *
 * Created: 2024-12-14 오전 3:17:23
 * Author : ghdtj
 */

#define F_CPU 14745600UL
#include "lcd.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/portpins.h>

#define	   DO    1908 // 262Hz (3817us) 1908us
#define	   RE    1700 // 294Hz (3401us) 1701us
#define	   MI    1515 // 330Hz (3030us) 1515us
#define	   FA    1432 // 349Hz (2865us) 1433us
#define	   SOL   1275 // 370Hz (2703us) 1351us
#define	   LA    1136 // 440Hz (2273us) 1136us
#define	   SI    1012 // 494Hz (2024us) 1012us
#define    HIGH_DO    956 // 523Hz (1912us) 956us
#define    HIGH_RE    852 // 587Hz (1704us) 852us
#define    HIGH_MI    759 // 659Hz (1517us) 759us

#define SetRow(x) PORTC |= 0xF0; PORTC &= ~(1<<((x-1)+PC4));

#define TRIG 6               // Trigger 핀 (출력 = PE6)
#define ECHO 7               // Echo 핀 (입력 = PE7)
#define SOUND_VELOCITY 340UL // 소리 속도 (m/s)

// FND 관련 함수
unsigned char Port_char[] = {
	0xc0,0xf9,0xa4,0xb0,0x99,
	0x92,0x82,0xd8,0x80,0x90,
	0x88,0x83,0xc4,0xa1,0x84,
	0x8e
}; // 애노드 공통

unsigned int Port_fnd[] = {0x1f,0x2f,0x4f,0x8f}; // FND0 ON, FND1 ON, FND2 ON, FND3 ON

void PORT_Init(void) {
	DDRF = 0xf0; // FND 제어위한 출력 설정(PF4 .. 7)
	DDRB = 0xff; // FND 제어위한 출력 설정(PB0 .. 7)
	DDRC = 0xF0; // PORTC 4 ~ 7 은 Row 선택위한 출력으로 설정
	// PORTC 0 ~ 3 은 col 값 읽기위한 입력으로 설정
}

void FND_Disp(int dec) {
	PORTF = Port_fnd[0]; PORTB = Port_char[(dec / 1000 % 10)]; _delay_ms(1); PORTB = 0xFF;
	PORTF = Port_fnd[1]; PORTB = Port_char[(dec / 100 % 10)]; _delay_ms(1); PORTB = 0xFF;
	PORTF = Port_fnd[2]; PORTB = Port_char[(dec / 10 % 10)]; _delay_ms(1); PORTB = 0xFF;
	PORTF = Port_fnd[3]; PORTB = Port_char[(dec / 1 % 10)]; _delay_ms(1); PORTB = 0xFF;
	PORTF = 0x00;
}

int getNum(unsigned char keyCode) {
	unsigned char keyNum[16] = {1,2,3,'A', 4,5,6,'B', 7,8,9,'C', '*',0,'#','D'};
	if(keyCode > 16)
	return -1;
	return keyNum[keyCode - 1];
}

int comparePassword(unsigned char* input, unsigned char* preset, unsigned char length) {
	for (int i = 0; i < length; i++) {
		if (input[i] != preset[i]) return 0; // 비밀번호 불일치
	}
	return 1; // 비밀번호 일치
}

// Buzzer 관련 함수
unsigned int sound_list[10] = {HIGH_MI, DO, RE, MI, FA, SOL, LA, SI, HIGH_DO, HIGH_RE};

void Buzzer_Init(void) {
	DDRG |= (1<<PG4);
	PORTG &= ~(1 << PG4); // 초기 상태: 부저 OFF
}

void myDelay_us(unsigned int delay){
	int i;
	for(i=0; i<delay; i++){
		_delay_us(1);
	}
}

void SSound(int time) {
	int i, tim;
	tim = 25000 / time;     //0.05 초
	for(i=0; i<tim; i++){
		PORTG |= (1<<PG4); //buzzer on, PORTG의 4번 핀 off(out 1)
		myDelay_us(time);
		PORTG &= ~(1<<PG4); //buzzer off, PORTG의 4번 핀 on(on 1)
		myDelay_us(time);
	}
	PORTG |= (1<<PG4); // buzzer off, PORTG의 4번 핀 off(out 0)
}

void correct_sound(void){
	SSound(DO);
	SSound(MI);
	SSound(SOL);
}

void wrong_sound(void){
	for (int i=0; i<3; i++){
		SSound(MI);
		_delay_ms(5);
	}
}

// 경고음 시작 함수
void startSiren(void) {
	TCCR1A = 0x00;       // CTC 모드
	TCCR1B = (1 << WGM12) | (1 << CS11); // 분주비 8, CTC 모드
	OCR1A = 1474;        // 1kHz (F_CPU / (2 * 8 * 1kHz) - 1)
	TIMSK |= (1 << OCIE1A); // 비교 일치 인터럽트 활성화
}

// 경고음 정지 함수
void stopSiren(void) {
	TCCR1B = 0x00;       // 타이머 정지
	PORTG &= ~(1 << PG4); // 부저를 끔
}

// 타이머 비교 일치 인터럽트 서비스 루틴
ISR(TIMER1_COMPA_vect) {
	PORTG ^= (1 << PG4); // PG4 핀 토글 (부저 진동)
}


// Servo_Motor 초기화
void Servo_Init(void) {
	DDRE |= (1 << PE4); // PORTE4 핀을 출력으로 설정
}

// Timer3 초기화 (서보 모터 제어용)
void Timer3_Init(void) {
	TCCR3A = (1 << WGM31) | (1 << COM3B1); // Fast PWM 모드, 비반전 출력
	TCCR3B = (1 << WGM32) | (1 << WGM33) | (1 << CS31); // 분주비 8
	ICR3 = 36864; // TOP 값 (PWM 주기 20ms, 50Hz)
	OCR3B = 3010; // 초기값 (1.5ms, 0도 위치)
}

// 초음파 센서 초기화
void Ultrasonic_Init(void) {
	DDRE |= (1 << TRIG); // TRIG = 출력
	DDRE &= ~(1 << ECHO); // ECHO = 입력
}

// 초음파 거리 측정 함수
unsigned int measure_distance(void) {
	unsigned int distance;

	TCCR1B = 0x03; // Timer/Counter1 클럭 설정: 64분주
	PORTE &= ~(1 << TRIG); // Trig LOW
	_delay_us(10);
	PORTE |= (1 << TRIG);  // Trig HIGH
	_delay_us(10);
	PORTE &= ~(1 << TRIG); // Trig LOW

	while (!(PINE & (1 << ECHO))); // Echo HIGH 대기
	TCNT1 = 0x0000; // Timer 초기화

	while (PINE & (1 << ECHO)); // Echo LOW 대기
	TCCR1B = 0x00; // Timer 정지

	distance = (unsigned int)(SOUND_VELOCITY * (TCNT1 * 4 / 2) / 1000); // mm 단위 거리 계산
	return distance;
}

// 전역 변수 선언
unsigned char new_password[10] = {10,10,10,10,10,10,10,10,10,10}; // 새 비밀번호
unsigned char new_char_count = 0; // 새 비밀번호 입력 문자 개수
int password_change_mode = 0;     // 1: 비밀번호 변경 모드 활성화
int state = 0;                    // 시스템 상태 (0: 비활성화, 1: 활성화)

// ISR 함수
ISR(INT1_vect) {
	if (!password_change_mode) { // 비밀번호 변경 모드가 비활성화된 경우에만 동작
		password_change_mode = 1;
		state = 1; // 비밀번호 입력 활성화
		LCD_Clear();
		LCD_Pos(0, 0);
		LCD_Str("Set New Password:");
		new_char_count = 0;
		for (int i = 0; i < 10; i++) {
			new_password[i] = 10; // 새 비밀번호 초기화
		}
	}
}

// 인터럽트 초기화 함수
void interrupt_init(void) {
	EIMSK = (1 << INT1);  // INT1 활성화
	EICRA = (1 << ISC11); // 하강 에지에서 트리거 (ISC11 = 1, ISC10 = 0)
	DDRD &= ~(1 << PD1);  // PORTD.1을 입력으로 설정
	PORTD |= (1 << PD1);  // 내부 풀업 활성화
	sei();                // 전역 인터럽트 허용
}

// USART 초기화 함수
void USART_Init(unsigned int ubrr) {
    UBRR1H = (unsigned char)(ubrr >> 8);  // 상위 8비트 설정
    UBRR1L = (unsigned char)ubrr;        // 하위 8비트 설정

    UCSR1B = (1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1);  // 송수신 허가 및 수신 인터럽트 활성화
    UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);  // 8비트 데이터, 1 스톱 비트
}

// UART 데이터 송신 함수
void USART_Transmit(unsigned char data) {
    while (!(UCSR1A & (1 << UDRE1)));  // 송신 가능 대기
    UDR1 = data;  // 데이터 전송
}

// 문자열 송신 함수
void USART_TransmitString(const char* str) {
    while (*str) {
        USART_Transmit(*str++);
    }
    USART_Transmit('\n');  // 엔터 전송
}

// UART 수신 인터럽트 핸들러
ISR(USART1_RX_vect) {
    unsigned char received = UDR1;  // 수신 데이터 읽기
}

void main() {
	unsigned char sel_row, keypad_code = 0, hkeypad_code = 0, col;
	unsigned char password_input[10] = {10,10,10,10,10,10,10,10,10,10}; // 입력 비밀번호
	unsigned char preset_password[10] = {2,3,5,6,10,10,10,10,10,10}; // 미리 설정된 비밀번호
	unsigned char char_count = 0;
	int fnd_value = 0;
	int key_hold_time = 0;
	int error_count = 0;
	int check_count = 0;

	unsigned int distance_mm;
	unsigned int distance_cm;
	char buffer[16]; // LCD에 표시할 문자열 저장 버퍼

	LCD_Init();
	LCD_Clear();
	LCD_Pos(0, 0);
	LCD_Str("Start Setting");

	PORT_Init();
	interrupt_init();
	Buzzer_Init();
	Servo_Init();
	Timer3_Init();
	Ultrasonic_Init();
	
	unsigned int ubrr = 7;
	USART_Init(ubrr);  // USART 초기화

	sei();
	SSound(DO);

	while (1) {
		if (state == 1 && !password_change_mode) {
			FND_Disp(fnd_value);
		}

		keypad_code = 0xFF;
		for (sel_row = 1; sel_row <= 4; sel_row++) {
			SetRow(sel_row);
			_delay_us(10);
			col = (PINC & 0x0F);
			switch (col) {
				case 0x01: keypad_code = (sel_row - 1) * 4 + 1; break;
				case 0x02: keypad_code = (sel_row - 1) * 4 + 2; break;
				case 0x04: keypad_code = (sel_row - 1) * 4 + 3; break;
				case 0x08: keypad_code = (sel_row - 1) * 4 + 4; break;
			}
		}

		if ((keypad_code != 0xFF) && (hkeypad_code != keypad_code)) {
			unsigned char key_value = getNum(keypad_code);

			// state = 1 : '#' 이 눌려서 비밀번호 입력이 되는 상태
			if (state == 1 && !password_change_mode) {
				if (key_value >= 0 && key_value <= 9) {
					// Buzzer
					SSound(sound_list[key_value]);
					
					// LCD에 키패드 숫자 출력
					LCD_Pos(1, char_count);
					LCD_CHAR(key_value + '0');
					password_input[char_count] = key_value;
					char_count++;
					if (char_count >= 10) char_count = 0;

					// FND에 현재 입력된 숫자 표시
					fnd_value %= 1000;
					fnd_value *= 10;
					fnd_value += getNum(keypad_code);
				}
				
				if (key_value == 35) { // '#' 키로 비밀번호 확인
					// 비밀 번호 일치하였을 때
					if (comparePassword(password_input, preset_password, 10)) {
						correct_sound();
						LCD_Clear();
						LCD_Pos(0, 0);
						LCD_Str("Password Match");
						// -90도 위치로 이동 (0.8ms)
						OCR3B = 1390; // (0.8ms / 20ms) * 36864 ≈ 1390
						error_count = 0;
						check_count = 0;
						_delay_ms(2000);
						while (check_count <= 6){
							distance_mm = measure_distance(); // 거리 측정 (mm 단위)
							distance_cm = distance_mm / 10;   // cm 단위로 변환
							LCD_Clear(); // LCD 화면 초기화
							snprintf(buffer, sizeof(buffer), "Distance: %dcm", distance_cm); // 거리 값 포맷
							LCD_Str((Byte*)buffer); // 거리 값 출력
							_delay_ms(500); // 0.5초 간격
							if (distance_cm < 7)
							{
								check_count += 1;
							}
							else{
								check_count = 0;
							}
							
							// PORTD.1 눌림 확인
							if (!(PIND & 0x02)) { // PORTD.1이 LOW(눌림 상태)인지 확인
								password_change_mode = 1; // 비밀번호 변경 모드 활성화
								state = 1; // 활성화 상태로 변경
								new_char_count = 0; // 새 비밀번호 입력 초기화
								for (int i = 0; i < 10; i++) new_password[i] = 10;

								LCD_Clear();
								LCD_Pos(0, 0);
								LCD_Str("Set New Password:");

								// 비밀번호 변경 모드
								while (password_change_mode) {
									keypad_code = 0xFF;
									for (sel_row = 1; sel_row <= 4; sel_row++) {
										SetRow(sel_row);
										_delay_us(10);
										col = (PINC & 0x0F);
										switch (col) {
											case 0x01: keypad_code = (sel_row - 1) * 4 + 1; break;
											case 0x02: keypad_code = (sel_row - 1) * 4 + 2; break;
											case 0x04: keypad_code = (sel_row - 1) * 4 + 3; break;
											case 0x08: keypad_code = (sel_row - 1) * 4 + 4; break;
										}
									}
									if ((keypad_code != 0xFF) && (hkeypad_code != keypad_code)) {
										unsigned char key_value = getNum(keypad_code);
										if (key_value >= 0 && key_value <= 9) {
											LCD_Pos(1, new_char_count);
											LCD_CHAR(key_value + '0');
											new_password[new_char_count] = key_value;
											new_char_count++;
											if (new_char_count >= 10) new_char_count = 0;
										}

										if (key_value == 35) { // '#' 키로 비밀번호 변경 완료
											for (int i = 0; i < 10; i++) {
												preset_password[i] = new_password[i];
											}
											password_change_mode = 0;
											LCD_Clear();
											LCD_Pos(0, 0);
											LCD_Str("Password Changed");
											_delay_ms(2000);
										}
									}
									hkeypad_code = keypad_code;
								}

								break; // while 루프 종료
							}
						}
						// +90도 위치로 이동 (2.4ms)
						OCR3B = 3010; // (2.4ms / 20ms) * 36864 ≈ 4410
						USART_TransmitString("Arrived\n");
						} else { // 비밀번호 불일치 하였을 때
						wrong_sound();
						LCD_Clear();
						LCD_Pos(0, 0);
						LCD_Str("Wrong Password");
						error_count += 1;
					}
					_delay_ms(2000);
					char_count = 0;
					for (int i = 0; i < 10; i++) password_input[i] = 10; // 초기화
					fnd_value = 0; // FND 값 초기화
				}
			}
			
			// '#' 이 눌렸을 때 상태 파악
			if (key_value == 35 && !password_change_mode) { // '#' 키로 상태 변경
				state = !state;
				if (state == 1) {
					LCD_Clear();
					LCD_Pos(0, 0);
					LCD_Str("Enter Password:");
					} else {
					LCD_Clear();
					LCD_Pos(0, 0);
					LCD_Str("Deactivated");
					for (int i = 0; i < 10; i++) password_input[i] = 10; // 초기화
					
					if (error_count >= 3) { // 비밀번호 3회 이상 틀릴 시 경고문 출력
						LCD_Clear();
						LCD_Pos(0, 0);
						LCD_Str("WARNING!!!");
						
						USART_TransmitString("WARNING!!!\n");
						
						startSiren();
					}
				}
			}
		}
		hkeypad_code = keypad_code;
	}
}