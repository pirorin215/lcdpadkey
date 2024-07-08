static __attribute__((section (".noinit")))char losabuf[4096];

#include <lcd.h>
#include <math.h>
#include <stdio.h>
#include <float.h>
#include <string.h>
#include <stdarg.h>

#include <hardware/i2c.h>
#include <hardware/adc.h>
#include <hardware/rtc.h>
#include <hardware/gpio.h>
#include <hardware/sync.h>
#include <hardware/clocks.h>
#include <hardware/interp.h>
#include <hardware/flash.h>
#include <hardware/watchdog.h>

#include <pico/time.h>
#include <pico/types.h>
#include <pico/stdlib.h>
#include <pico/bootrom.h>
#include <pico/i2c_slave.h>
#include <pico/binary_info.h>
#include <pico/util/datetime.h>
#include <pico/bootrom/sf_table.h>

#define GYRO_6AXIS            // ジャイロセンサ利用フラグ

#ifdef GYRO_6AXIS
#include <QMI8658.h>            // ジャイロセンサライブラリ
#endif

#include "w.h"
#include "lib/draw.h"
#include "img/font34.h"
#include "CST816S.h"           // LCDタッチ液晶ライブラリ

typedef struct {
  char mode[8];
  datetime_t dt;
  uint8_t theme;
  uint8_t editpos;
  uint8_t BRIGHTNESS;

  bool sensors;
  bool gyrocross;
  bool bender;
  bool SMOOTH_BACKGROUND;
  bool INSOMNIA;
  bool DYNAMIC_CIRCLES;
  bool DEEPSLEEP;
  bool is_sleeping;
  bool highpointer;
  bool alphapointer;
  bool clock;
  bool rotoz;
  bool rota;
  float fspin;
  uint8_t pstyle;
  int8_t spin;
  uint8_t texture;
  uint8_t configpos;
  uint8_t conf_bg;
  uint8_t conf_phour;
  uint8_t conf_pmin;
  uint8_t scandir;
  bool dither;
  uint8_t dummy;
  uint8_t save_crc;
} LOSA_t;

// i2c通信用
#define I2C_BUFFER_SIZE 10  // リングバッファのサイズ

typedef struct {
	uint8_t click;
	int8_t pointer_x;
	int8_t pointer_y;
	int8_t wheel_h;
	int8_t wheel_v;
} simple_pointer_data_t;

typedef struct {
    simple_pointer_data_t buffer[I2C_BUFFER_SIZE];
    int head;
    int tail;
    int count;
} ring_buffer_t;

volatile ring_buffer_t i2c_ring_buffer = {0};

// 座標構造体
typedef struct {
  int16_t x;
  int16_t y;
}axis_t;
const axis_t axis_0 = {0};	// 初期化用構造体

// スクロール状態
typedef struct {
  int count;
  int sum;
  int last;
}scroll_t;

// 設定画面呼び出し条件
typedef struct {
	int cnt;              // 設定画面の操作カウント
	uint32_t time;        // 設定画面の操作開始時刻
}sg_trigger_t;

// I2C通信

#define LCDPADKEY_NONE_CLICK   0
#define LCDPADKEY_CLICK_LEFT   1
#define LCDPADKEY_CLICK_RIGHT  1 << 1
#define LCDPADKEY_CLICK_MIDDLE 1 << 2
#define LCDPADKEY_NOCHANGE     1 << 3
#define LCDPADKEY_IGNORE       1 << 4
#define LCDPADKEY_READ_REG     1 << 5

// タッチ操作状態
typedef enum {
  MODE_NONE,
  MODE_TOUCHING,
  MODE_TOUCH_RELEASE,
  MODE_SCROLL_Y,
  MODE_SCROLL_X,
  MODE_DRAG,
  MODE_L_CLICK,
  MODE_R_CLICK,
  MODE_GYRO,
  MODE_GYRO_SCROLL,
} TOUCH_MODE;

// 設定項目のリスト
typedef enum {
	SG_SLEEP,
	SG_DRUG_DIR,
	SG_DRUG_LEN,
	SG_R_CLICK_DIR,
	SG_R_CLICK_LEN,
	SG_SCROLL_Y_DIR,
	SG_SCROLL_Y_LEN,
	SG_SCROLL_Y_REV,
	SG_SCROLL_X_DIR,
	SG_SCROLL_X_LEN,
	SG_SCROLL_X_REV,
	SG_TITLE_SPEED,
	SG_TAP_DRAG,
	SG_ACC_LIMIT,
	SG_ACC_RATE,
	SG_GYRO,
	SG_GYRO_SCROLL,
	SG_VIBRATION,
	SG_GAME,
	SG_EXIT,
	SG_NUM,
} SG_ITEM;

// 特殊操作の液晶画面位置（上下左右)
typedef enum {
	DIR_TOP,
	DIR_BOTTOM,
	DIR_RIGHT,
	DIR_LEFT,
	DIR_NUM,
} SG_DIR;

static LOSA_t* plosa=(LOSA_t*)losabuf;
#define LOSASIZE (&plosa->dummy - &plosa->theme)

int16_t screensaver=100;
bool deepsleep=false;

volatile uint8_t flag_touch = 0;
extern uint8_t LCD_RST_PIN;

uint8_t* b0=NULL;
uint32_t* b1=NULL;

//one button /
#define QMIINT1 23
#define CBUT_TOUCH 16
uint8_t CBUT0 = 22;
bool fire_pressed=false;

bool fire=false;
bool ceasefire=false;

static int LCD_ADDR = 0x6b;

#define MS 1000
#define US 1000000
#define BUTD 500  // delay between possible button presses (default: 500, half of a second)
uint32_t rebootcounter = 0;
uint32_t button0_time=0;

volatile uint8_t flag_event = 0;

#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 240

#define TRIGGER_SG_TIME         1000 // 設定画面を出す操作時間
#define TRIGGER_SG_LEFT           30 // 設定画面を出す操作のタッチ位置 LEFT
#define TRIGGER_SG_RIGHT         190 // 設定画面を出す操作のタッチ位置 RIGHT

#define SG_CLICK_RELEASE_COUNT_LIMIT  5 // クリック判定
#define TOUCH_START_MSEC_LIMIT   100 // 新しいタッチ開始の判定msec
#define DRAG_START_MSEC          150 // ダブルタップドラッグの判定msec
#define DRAG_UNDER_LIMIT_MSEC    160 // ドラッグ開始から解除までの最低msec

#define FLASH_TARGET_OFFSET 0x1F0000 // W25Q16JVの最終ブロック(Block31)のセクタ0の先頭アドレス

const char POS_X[3] = { 40, 40, 40};
const char POS_Y[3] = { 40, 60, 80};

char g_text_buf[3][128] = {{0}};
char g_old_text_buf[3][128] = {{0}};

#define TITLE_X   30
#define TITLE_Y  130

///////////////////////////////
// 設定画面系
///////////////////////////////
const uint16_t COLOR_DRAG     = YELLOW;
const uint16_t COLOR_R_CLICK  = RED;
const uint16_t COLOR_SCROLL_Y = BLUE;
const uint16_t COLOR_SCROLL_X = GREEN;

#define RENZOKU_TOUCH_MSEC_LIMIT   500 // 連続入力の判定msec
#define SG_ITEMNAME_X   10
#define SG_ITEMNAME_Y  100

#define SG_VALUE_X      10
#define SG_VALUE_Y     140

#define SG_ITEM_HEIGHT  60
#define SG_ITEM_WIDTH  115

#define CHECK_NUMBER_SG 123
#define VIBRATION_PIN 18

// 設定画面の設定値のフレーム
int SG_FRAME_VALUE[4] = {88, 137, 88 + 100,  137 + 25};

// 設定画面のボタンのフレーム
int SG_FRAME_DOWN[4] =  { 30,  30,   0 + SG_ITEM_WIDTH     ,  30 + SG_ITEM_HEIGHT};
int SG_FRAME_UP  [4] =  {125,  30, 125 + SG_ITEM_WIDTH - 30,  30 + SG_ITEM_HEIGHT};
int SG_FRAME_PREV[4] =  { 30, 180,   0 + SG_ITEM_WIDTH     , 180 + SG_ITEM_HEIGHT};
int SG_FRAME_NEXT[4] =  {125, 180, 125 + SG_ITEM_WIDTH - 30, 180 + SG_ITEM_HEIGHT};

int SG_FRAME_RESET[4] =  { 40, 120,  60 + SG_ITEM_WIDTH + 20, 120 + SG_ITEM_HEIGHT - 10};

uint8_t g_sg_data[SG_NUM];	// 設定データ保存用

float acc[3], gyro[3];		// ジャイロセンサ
unsigned int tim_count = 0;	// ジャイロセンサ


/** フラッシュデータの初期化 **/
void init_sg(void) {
	g_sg_data[SG_SLEEP]		= 4;
	g_sg_data[SG_TITLE_SPEED]	= 4;
	g_sg_data[SG_DRUG_DIR]		= DIR_TOP;
	g_sg_data[SG_DRUG_LEN]		= 55;
	g_sg_data[SG_R_CLICK_DIR]	= DIR_LEFT;
	g_sg_data[SG_R_CLICK_LEN]	= 55;
	g_sg_data[SG_SCROLL_Y_DIR]	= DIR_RIGHT;
	g_sg_data[SG_SCROLL_Y_LEN]	= 55;
	g_sg_data[SG_SCROLL_Y_REV]	= 0;
	g_sg_data[SG_SCROLL_X_DIR]	= DIR_BOTTOM;
	g_sg_data[SG_SCROLL_X_LEN]	= 55;
	g_sg_data[SG_SCROLL_X_REV]	= 0;
	g_sg_data[SG_ACC_LIMIT]		= 12;
	g_sg_data[SG_ACC_RATE]		= 1;
	g_sg_data[SG_TAP_DRAG]		= 1;
	g_sg_data[SG_GYRO]		= 0;
	g_sg_data[SG_GYRO_SCROLL]	= 0;
	g_sg_data[SG_VIBRATION]		= 1;
	g_sg_data[SG_GAME]		= 0;
	g_sg_data[SG_EXIT]		= CHECK_NUMBER_SG;
}

bool reserved_addr(uint8_t addr) {
  return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

bool i2c_scan(){
	printf("\nI2C Bus Scan \n");
	bool b_cst816_enable = false;
	printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");
	for (int addr = 0; addr < (1 << 7); ++addr) {
			if (addr % 16 == 0) {					printf("%02x ", addr);			}
			int ret;
			uint8_t rxdata;
			if (reserved_addr(addr))
					ret = PICO_ERROR_GENERIC;
			else
					ret = i2c_read_blocking(I2C_PORT, addr, &rxdata, 1, false);

			if(ret >= 0 && addr == CST816_ADDR){
				b_cst816_enable = true;
				CBUT0 = CBUT_TOUCH;
				LCD_RST_PIN = 13;
			}
			printf(ret < 0 ? "." : "@");
			printf(addr % 16 == 15 ? "\n" : "  ");
	}
	return b_cst816_enable;
}

void gpio_callback(uint gpio, uint32_t events) {
	if(events&GPIO_IRQ_EDGE_RISE){
		if(gpio==CBUT0){ceasefire=true;fire_pressed=false;rebootcounter=0;}
		if(gpio==QMIINT1){ deepsleep=false; }
		if(gpio==Touch_INT_PIN){
			deepsleep=false;
			flag_touch = 1;
		}
	}

	if(events&GPIO_IRQ_EDGE_FALL){
		if(gpio==CBUT0 && !fire && (((time_us_32()-button0_time)/MS)>=BUTD)){ceasefire=false;fire=true;button0_time = time_us_32();fire_pressed=true;}
	}

}

void draw_background() { }

void draw_gfx(){ }

void draw_text(){ }

void truncateString(char* text, int maxLength) {
  if (strlen(text) > maxLength) {
    text[maxLength] = '\0'; // 文字列をmaxLengthの長さに切り詰める
  }
}

/** フラッシュに設定保存 **/
static void save_sg_to_flash(void) {
	uint32_t ints = save_and_disable_interrupts(); // 割り込み無効にする
	flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE); // Flash消去
	flash_range_program(FLASH_TARGET_OFFSET, g_sg_data, FLASH_PAGE_SIZE); // Flash書き込み
	restore_interrupts(ints); // 割り込みフラグを戻す
}

/** フラッシュから設定読み込み **/
void load_sg_from_flash(void) {
	const uint8_t *flash_target_contents = (const uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET);

	for(int i=0; i < SG_NUM; i++ ) {
		g_sg_data[i] = flash_target_contents[i];
		printf("SG %2d = %d\r\n", i, g_sg_data[i]);
	}
}

/** フラッシュデータチェック **/
void check_sg(void) {
	if(g_sg_data[SG_EXIT] != CHECK_NUMBER_SG) {
		init_sg();
	}
}

void lcd_range_line_draw(uint16_t line_color, int dir, int len) {

	// 円の角を超えすぎるとマイコンが固まるので開始と終点を補正するための数値
	// 端から40pixelより下で四角で考えた場合に0と最大を指定すると落ちる
	int other_len = len <= 40 ? 40 - len : 0;

	switch(dir) {
		case DIR_TOP:
			lcd_line(other_len, len, SCREEN_WIDTH - other_len, len, line_color, 3);
			break;
		case DIR_BOTTOM:
			lcd_line(other_len, SCREEN_HEIGHT - len, SCREEN_WIDTH - other_len, SCREEN_HEIGHT - len, line_color, 3);
			break;
		case DIR_LEFT:
			lcd_line(len, other_len, len, SCREEN_HEIGHT - other_len, line_color, 3);
			break;
		case DIR_RIGHT:
			lcd_line(SCREEN_HEIGHT - len, other_len, SCREEN_HEIGHT - len, SCREEN_HEIGHT - other_len, line_color, 3);
			break;
	}
}

void lcd_title_set() {
	lcd_str(TITLE_X, TITLE_Y, "LCD PadKey", &Font24, CYAN, BLACK);
}

void lcd_text_draw(uint16_t lcd_color) {

	// タイトル表示		
	lcd_title_set();

	// 線を描画
	lcd_range_line_draw(COLOR_DRAG,     g_sg_data[SG_DRUG_DIR],     g_sg_data[SG_DRUG_LEN]);
	lcd_range_line_draw(COLOR_R_CLICK,  g_sg_data[SG_R_CLICK_DIR],  g_sg_data[SG_R_CLICK_LEN]);
	lcd_range_line_draw(COLOR_SCROLL_Y, g_sg_data[SG_SCROLL_Y_DIR], g_sg_data[SG_SCROLL_Y_LEN]);
	lcd_range_line_draw(COLOR_SCROLL_X, g_sg_data[SG_SCROLL_X_DIR], g_sg_data[SG_SCROLL_X_LEN]);

	// 文字列データを表示
	for(int i=0; i<sizeof(g_text_buf)/sizeof(*g_text_buf); i++ ) {
		if(strcmp(g_text_buf[i], g_old_text_buf[i]) != 0) {
			// 古いのを消す
			lcd_str(POS_X[i], POS_Y[i]+1, g_old_text_buf[i], &Font20, lcd_color, WHITE);
		}

		// 表示する
		lcd_str(POS_X[i], POS_Y[i]+1, g_text_buf[i], &Font20, WHITE, lcd_color);

		// 古い文字列をバックアップ
		sprintf(g_old_text_buf[i], "%s", g_text_buf[i]);
	}
}

void lcd_text_set(int row, uint16_t lcd_color, bool b_console, const char *format, ...) {
	char text_buf[128]; // 出力用のバッファ
	va_list args;  // 可変引数リスト
	va_start(args, format); // 可変引数リストの初期化
	vsnprintf(text_buf, sizeof(text_buf), format, args); // フォーマットに従って文字列をバッファに書き込む
	va_end(args); // 可変引数リストのクリーンアップ

	if(b_console) {
		printf("PLM %s\r\n", text_buf); // コンソールログ出力
	}
		
	truncateString(text_buf, 12); // LCD表示用に長さを制限
	
	sprintf(g_text_buf[row-1], "%s", text_buf); // 表示配列にセット
}

void send_pointer() {
	volatile simple_pointer_data_t* i2c_tmp = &i2c_ring_buffer.buffer[i2c_ring_buffer.tail];

	uint8_t *raw_buf = (uint8_t *)i2c_tmp;
	i2c_write_blocking(i2c0, LCD_ADDR, &raw_buf[0], 1, true);
	for (int i = 1; i < 5; i++) {
		i2c_write_blocking(i2c0, LCD_ADDR, &raw_buf[i], 1, true);
	}
	
	i2c_ring_buffer.tail = (i2c_ring_buffer.tail + 1) % I2C_BUFFER_SIZE;
	i2c_ring_buffer.count--;
}

void recv_event(i2c_inst_t *i2c) {
	uint8_t cmd = i2c_read_byte_raw(i2c);
	
	switch(cmd) {
		case LCDPADKEY_READ_REG:
			flag_event=1;
			break;
	}
	while(true) {
		uint8_t b = i2c_read_byte_raw(i2c);
		if(b == 0) {
			break;
		}
	}
}

static simple_pointer_data_t i2c_buf_ignore = {0}; // I2Cの無視応答用

void send_event() {
	if(flag_event) {
		flag_event = 0;
		if (i2c_ring_buffer.count == 0) {
			// バッファが空の場合は無視データを応答
			i2c_buf_ignore.click = LCDPADKEY_IGNORE;
			uint8_t *raw_buf_tmp = (uint8_t *)&i2c_buf_ignore;
			for (int i = 0; i < 5; i++) {
				i2c_write_blocking(i2c0, LCD_ADDR, &raw_buf_tmp[i], 1, true);
			}
			return;
		}
		send_pointer();
	}
}

static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
	switch (event) {
		case I2C_SLAVE_RECEIVE: // master has written some data
			recv_event(i2c);
			break;
		case I2C_SLAVE_REQUEST: // master is requesting data
			send_event();
			break;
		case I2C_SLAVE_FINISH: // master has signalled Stop / Restart
			  break;
		default:
			  break;
	}
}

void i2c_data_set(uint8_t click, int x, int y, int h, int v) {
	if(click == LCDPADKEY_NOCHANGE && x == 0 && y == 0 && h == 0 && v == 0) {
		return;
	}

	if (i2c_ring_buffer.count == I2C_BUFFER_SIZE) {
	    // バッファが満杯の場合、最も古いデータを上書き
	    i2c_ring_buffer.tail = (i2c_ring_buffer.tail + 1) % I2C_BUFFER_SIZE;
	} else {
	    i2c_ring_buffer.count++;
	}
	
	volatile simple_pointer_data_t* i2c_tmp = &i2c_ring_buffer.buffer[i2c_ring_buffer.head];
	i2c_tmp->click = click;
	i2c_tmp->pointer_x = x;
	i2c_tmp->pointer_y = y;
	i2c_tmp->wheel_h = h;
	i2c_tmp->wheel_v = v;
	
	printf("i2c_data_set click=%u i2c_tmp->click=%u x:%d y:%d h:%d v:%d\r\n", click, i2c_tmp->click, x, y, h, v);
	
	i2c_ring_buffer.head = (i2c_ring_buffer.head + 1) % I2C_BUFFER_SIZE;
	flag_event = 1;
}

/** lcd_circle の範囲ガード **/
void lcd_circle_guard(uint16_t x, uint16_t y, uint16_t radius, uint16_t color, uint16_t ps, bool fill) {

	x = x < 0 + radius            ? radius                  : x;
	x = x > SCREEN_WIDTH  - radius ? SCREEN_WIDTH  - radius : x;
	
	y = y < 0 + radius            ? radius                  : y;
	y = y > SCREEN_HEIGHT - radius ? SCREEN_HEIGHT - radius : y;

	lcd_circle(x, y, radius, color, ps, fill);
}

// グローバル変数
volatile bool vibration_active = false;

void vibration_on() {
	printf("vibration_on\n");
        gpio_put(VIBRATION_PIN, 1);
}

void vibration_off() {
	printf("vibration_off\n");
        gpio_put(VIBRATION_PIN, 0);
}

void set_timer(uint32_t ms, int64_t (*callback)(alarm_id_t, void*), void *user_data) {
    add_alarm_in_ms(ms, callback, user_data, false);
}

int64_t stop_vibration_callback(alarm_id_t id, void *user_data) {
    vibration_off();
    vibration_active = false;
    return 0;
}

void trigger_vibration(int timer) {
	if (g_sg_data[SG_VIBRATION] >= 1) {
		if (!vibration_active) {
		    int strength = 1; // 固定
		    vibration_on(strength);
		    vibration_active = true;
		    
		    int *itmp;
		    set_timer(timer, stop_vibration_callback, itmp);
		}
	}
}

/** 中心点を光らせる処理 **/
int64_t center_point_flash_callback(alarm_id_t id, void *user_data) {
	int16_t color = (rand() % 65535) + 1;
	
	lcd_circle_guard(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2, 4, color, 1, true);

	printf("center_point_flash_callback color=%d\r\n", color);

    	return 0;
}

void center_point_flash(int count) {
	printf("center_point_flash count=%d\r\n", count);
	for(int i=0; i < count; i++ ) {
		printf("center_point_flash i=%d\r\n", i);
		int *itmp;
		set_timer(50*i, center_point_flash_callback, itmp);
	}	
}

/** 初期化処理 **/
void init() {
	sleep_ms(100);  // "Rain-wait" wait 100ms after booting (for other chips to initialize)
	rtc_init();
	stdio_init_all();

	// USB接続確立待ち
	////stdio_usb_init();
	//int i = 0;
	//while (i++ < 10) {
	//	if (stdio_usb_connected())
	//		break;
	//	sleep_ms(250);
	//}

	plosa->dummy=0;

	if(plosa->BRIGHTNESS < 10)plosa->BRIGHTNESS = 10;
	if(plosa->BRIGHTNESS > 100)plosa->BRIGHTNESS = 100;

	// バイブレーションモーター用PIN
	gpio_init(VIBRATION_PIN);
        gpio_set_dir(VIBRATION_PIN, GPIO_OUT);
        gpio_put(VIBRATION_PIN, 0);

	// I2C Master(LCDとの通信）
	i2c_init(I2C_PORT, 100 * 1000);

	gpio_set_function(DEV_SDA_PIN, GPIO_FUNC_I2C);
	gpio_set_function(DEV_SCL_PIN, GPIO_FUNC_I2C);
	gpio_pull_up(DEV_SDA_PIN);
	gpio_pull_up(DEV_SCL_PIN);
   
	// I2C Slave(メインのpico側への通信） 
	i2c_slave_init(i2c0, 0x0B, &i2c_slave_handler);
	gpio_set_function(16, GPIO_FUNC_I2C);
	gpio_set_function(17, GPIO_FUNC_I2C);
	gpio_pull_up(16);
	gpio_pull_up(17);
	
	// I2CスキャンでLCD見つかるまでループ	
	while(true) {
		if(i2c_scan()) {
			break;
		}
		sleep_ms(1000);
	}

	lcd_init();

	plosa->scandir&=0x00; // 画面上向き
	lcd_setatt(plosa->scandir&0x03);

	lcd_make_cosin();
	lcd_set_brightness(plosa->BRIGHTNESS);
	printf("%02d-%02d-%04d %02d:%02d:%02d [%d]\n",plosa->dt.day,plosa->dt.month,plosa->dt.year,plosa->dt.hour,plosa->dt.min,plosa->dt.sec,plosa->dt.dotw);
	printf("mode='%s'\n",plosa->mode);
	printf("LOSASIZE=%d\n",LOSASIZE);
	b0 = malloc(LCD_SZ);
	b1 = (uint32_t*)b0;
	if(b0==0){printf("b0==0!\n");}
	uint32_t o = 0;
	lcd_setimg((uint16_t*)b0);

	printf("init realtime clock\n");
	rtc_set_datetime(&plosa->dt);
	printf("init realtime clock done\n");

	gpio_init(QMIINT1);
	gpio_set_dir(QMIINT1,GPIO_IN);
	gpio_set_irq_enabled_with_callback(QMIINT1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

	gpio_init(Touch_INT_PIN);
	gpio_pull_up(Touch_INT_PIN);
	gpio_set_dir(Touch_INT_PIN,GPIO_IN);
	gpio_set_irq_enabled(Touch_INT_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
	CST816S_init(CST816S_Point_Mode);

#ifdef GYRO_6AXIS
    	QMI8658_enableWakeOnMotion();
	QMI8658_init();sleep_ms(2500);printf("init done\n");
#endif
}

/** ジャイロセンサ用 **/
int16_t get_acc02f(float f0, float f1, float FACT){
  switch(plosa->scandir){
    case 0: return (int16_t)(f0/FACT);break;
    case 1: return (int16_t)(f1/FACT);break;
    case 2: return (int16_t)(f0/-FACT);break;
    case 3: return (int16_t)(f1/-FACT);break;
  }
}
int16_t get_acc12f(float f0, float f1, float FACT){
  switch(plosa->scandir){
    case 0: return (int16_t)(f1/FACT);break;
    case 1: return (int16_t)(f0/-FACT);break;
    case 2: return (int16_t)(f1/-FACT);break;
    case 3: return (int16_t)(f0/FACT);break;
  }
}
int16_t get_acc02(float f0, float f1){return get_acc02f(f0,f1,25.0f);}
int16_t get_acc12(float f0, float f1){return get_acc12f(f0,f1,25.0f);}
int16_t get_acc0(){return get_acc02(acc[0]*1.5f,acc[1]*1.5f);}
int16_t get_acc1(){return get_acc12(acc[0]*1.5f,acc[1]*1.5f);}
/************************************/

/** 画面方向に合わせてX,Y座標を補正して取得 **/
axis_t axis_rotate() {
	CST816S_Get_Point(); // 座標を取得
	axis_t axis_cur;
	axis_cur.x = (int16_t)Touch_CTS816.x_point;
	axis_cur.y = (int16_t)Touch_CTS816.y_point;
	if(plosa->scandir==1){
	      int16_t tt = axis_cur.x; axis_cur.x = axis_cur.y; axis_cur.y = tt;
	      axis_cur.y = LCD_H - axis_cur.y;
	}else if(plosa->scandir==2){
	      axis_cur.x = LCD_W - axis_cur.x;
	      axis_cur.y = LCD_H - axis_cur.y;
	}else if(plosa->scandir==3){
	      int16_t tt = axis_cur.x; axis_cur.x = axis_cur.y; axis_cur.y = tt;
	      axis_cur.x = LCD_W - axis_cur.x;
	}
	return axis_cur;
}

/** 押しはじめ判定 **/
bool isRangePress(axis_t axis_cur, int dir, int len) {
	bool bInRange = false;
	switch(dir) {
		case DIR_TOP:
			bInRange = axis_cur.y <= len;
			break;
		case DIR_BOTTOM:
			bInRange = axis_cur.y >= SCREEN_HEIGHT - len;
			break;
		case DIR_LEFT:
			bInRange = axis_cur.x <= len;
			break;
		case DIR_RIGHT:
			bInRange = axis_cur.x >= SCREEN_WIDTH - len;
			break;
	}
	if(bInRange) {
		printf("range press dir=%d len=%d\r\n", dir, len);
		return true;
	}
	return false;
}

int abs_value(int a, int b) {
	return abs(abs(a) - abs(b));
}

/** 前回と近い場所をタッチ判定 **/
bool isNearbyPoint(axis_t axis1, axis_t axis2, int delta) {
	return abs_value(axis1.x, axis2.x) < delta && abs_value(axis1.y, axis2.y) < delta;
}

/** 押し続け判定 **/
bool isKeepPress(int release_cnt, axis_t axis_touch, axis_t axis_cur, int dir, int len) {

	if(release_cnt > 20 && isNearbyPoint(axis_touch, axis_cur, 5)) {
		bool bInRange = false;
		switch(dir) {
			case DIR_TOP:
				bInRange = axis_cur.y <= len;
				break;
			case DIR_BOTTOM:
				bInRange = axis_cur.y >= SCREEN_HEIGHT - len;
				break;
			case DIR_LEFT:
				bInRange = axis_cur.x <= len;
				break;
			case DIR_RIGHT:
				bInRange = axis_cur.x >= SCREEN_WIDTH - len;
				break;
		}
		if(bInRange) {
			printf("keep press dir=%d len=%d\r\n", dir, len);
			return true;
		}
	}
	return false;
}


/** 座標の以前からの移動量の計算 **/
axis_t get_axis_delta(axis_t axis_cur, axis_t axis_old, double z) {

	axis_t axis_delta;
				
	axis_delta.x = axis_old.x == 0 ? 0 : axis_cur.x - axis_old.x;
	axis_delta.y = axis_old.y == 0 ? 0 : axis_cur.y - axis_old.y;
	
	axis_delta.x = ceil(axis_delta.x * z);
	axis_delta.y = ceil(axis_delta.y * z);

	return axis_delta;
}

/** 加速処理 **/
axis_t acc_axis_delta(axis_t axis_delta) {

	double delta_value = sqrt(pow(axis_delta.x, 2) + pow(axis_delta.y, 2));

	double acc = 1;
	if(delta_value < g_sg_data[SG_ACC_LIMIT]) {
		acc = 1;
	} else {
		acc = g_sg_data[SG_ACC_RATE];
	}
	printf("delta_value=%.1f acc=%.1f\n", delta_value, acc);

	axis_delta.x = ceil(axis_delta.x * acc);
	axis_delta.y = ceil(axis_delta.y * acc);

	return axis_delta;
}

void lcd_frame_set(int frame[], int16_t lcd_color, uint8_t ps) {
	lcd_frame(frame[0], frame[1], frame[2], frame[3], lcd_color, ps);
}

void lcd_button_frame_set(int frame[], int16_t color1, uint8_t ps1, int16_t color2, uint8_t ps2) {
	lcd_frame(frame[0]      , frame[1]      , frame[2],       frame[3],       color1, ps1);
	lcd_frame(frame[0] + ps1, frame[1] + ps1, frame[2] - ps1, frame[3] - ps1, color2, ps2);
}

char *get_sg_itemname(int sg_no) {
	static char tmps[32];
	int sz = sizeof(tmps);
	// 設定項目タイトル
	switch(sg_no) {
		case SG_SLEEP:		snprintf(tmps, sz, "%02d:SLEEP", sg_no); break;
		case SG_DRUG_DIR:	snprintf(tmps, sz, "%02d:DRUG DIR", sg_no); break;
		case SG_DRUG_LEN:	snprintf(tmps, sz, "%02d:DRUG LEN", sg_no); break;
		case SG_R_CLICK_DIR:	snprintf(tmps, sz, "%02d:R CLICK DIR", sg_no); break;
		case SG_R_CLICK_LEN:	snprintf(tmps, sz, "%02d:R CLICK LEN", sg_no); break;
		case SG_SCROLL_Y_DIR:	snprintf(tmps, sz, "%02d:SCROLL Y DIR", sg_no); break;
		case SG_SCROLL_Y_LEN:	snprintf(tmps, sz, "%02d:SCROLL Y LEN", sg_no); break;
		case SG_SCROLL_Y_REV:	snprintf(tmps, sz, "%02d:SCROLL Y REV", sg_no); break;
		case SG_SCROLL_X_DIR:	snprintf(tmps, sz, "%02d:SCROLL X DIR", sg_no); break;
		case SG_SCROLL_X_LEN:	snprintf(tmps, sz, "%02d:SCROLL X LEN", sg_no); break;
		case SG_SCROLL_X_REV:	snprintf(tmps, sz, "%02d:SCROLL X REV", sg_no); break;
		case SG_TITLE_SPEED:	snprintf(tmps, sz, "%02d:TITLE SPEED", sg_no); break;
		case SG_ACC_LIMIT:	snprintf(tmps, sz, "%02d:ACC LIMIT", sg_no); break;
		case SG_ACC_RATE:	snprintf(tmps, sz, "%02d:ACC RATE", sg_no); break;
		case SG_TAP_DRAG:	snprintf(tmps, sz, "%02d:TAP DRAG", sg_no); break;
		case SG_GYRO:		snprintf(tmps, sz, "%02d:GYRO SPEED", sg_no); break;
		case SG_GYRO_SCROLL:	snprintf(tmps, sz, "%02d:GYRO SCROLL", sg_no); break;
		case SG_VIBRATION:	snprintf(tmps, sz, "%02d:VIBRATION", sg_no); break;
		case SG_GAME:		snprintf(tmps, sz, "%02d:GAMEMODE", sg_no); break;
		case SG_EXIT:		snprintf(tmps, sz, "      EXIT"); break;
	}
	return tmps;
}                                  

/** 指定 座標がフレーム範囲に入って座標がフレーム範囲に入ってるかチェック **/
bool is_frame_touch(int frame[], axis_t axis_cur) {
	if(
		frame[0]   <= axis_cur.x && 
		frame[1]   <= axis_cur.y && 
		axis_cur.x <= frame[2] && 
		axis_cur.y <= frame[3]
	) {
		return true;
	}
	return false;
}

/** 設定画面描画 **/
void lcd_sg_draw(int sg_no) {

	lcd_clr(0x2222);
	
	printf("lcd_sg_draw 01 sg_no=%d\r\n", sg_no);
	printf("lcd_sg_draw 02 sg_no=%d gsi=%s\r\n", sg_no, get_sg_itemname(sg_no));

	// 設定項目名の表示
	lcd_str(SG_ITEMNAME_X, SG_ITEMNAME_Y, get_sg_itemname(sg_no), &Font20, CYAN, BLACK);
	
	printf("lcd_sg_draw 03\r\n");

	char tmps[32];
	sprintf(tmps, "Value ");

	switch(sg_no) {
		case SG_SLEEP:
		case SG_TITLE_SPEED:
		case SG_TAP_DRAG:
		case SG_GYRO:
		case SG_GYRO_SCROLL:
		case SG_VIBRATION:
		case SG_ACC_LIMIT:
		case SG_ACC_RATE:
		case SG_GAME:
			if(g_sg_data[sg_no] == 0) {
				strcat(tmps, "OFF");
			} else {
				sprintf(tmps, "%s%d", tmps, g_sg_data[sg_no]);
			}
			break;
		case SG_DRUG_DIR:
		case SG_R_CLICK_DIR:
		case SG_SCROLL_Y_DIR:
		case SG_SCROLL_X_DIR:
			// 方向系の設定は文字に変換
			switch(g_sg_data[sg_no]) {
				case DIR_TOP:		strcat(tmps, "TOP"); break;
				case DIR_BOTTOM:	strcat(tmps, "BOTTOM"); break;
				case DIR_LEFT:		strcat(tmps, "LEFT"); break;
				case DIR_RIGHT:		strcat(tmps, "RIGHT"); break;
			}
			break;
		default:
			sprintf(tmps, "%s%d", tmps, g_sg_data[sg_no]);
			break;
	}
	
	if(sg_no != SG_EXIT) {
		// 設定値の枠
		lcd_frame_set(SG_FRAME_VALUE, RED, 1);
		// 設定値
		lcd_str(SG_VALUE_X, SG_VALUE_Y, tmps, &Font20, WHITE, BLACK);
	}
	
	// 設定ボタンの表示
	int px=8;
	int py=8;
	lcd_button_frame_set(SG_FRAME_DOWN, BLACK, 5, GRAY,  30);
	lcd_button_frame_set(SG_FRAME_UP  , BLACK, 5, GRAY,  30);
	
	if(sg_no != SG_EXIT) {
		lcd_str(SG_FRAME_DOWN[0] + px +  5, SG_FRAME_DOWN[1] + py, "DOWN",   &Font20, BLACK, WHITE);
		lcd_str(SG_FRAME_UP[0]   + px + 15, SG_FRAME_UP[1]   + py, "UP",     &Font20, BLACK, WHITE);
	} else {
		lcd_str(SG_FRAME_DOWN[0] + px +  0, SG_FRAME_DOWN[1] + py, "CANCEL", &Font16, BLACK, WHITE);
		lcd_str(SG_FRAME_UP[0]   + px +  0, SG_FRAME_UP[1]   + py, "SAVE",   &Font24, BLUE,   WHITE);

		lcd_button_frame_set(SG_FRAME_RESET, BLACK, 5, GRAY,  30);
		lcd_str(SG_FRAME_RESET[0]+ px +  0, SG_FRAME_RESET[1]+ py, "ALL RESET", &Font20, RED,   WHITE);
	}
	
	lcd_button_frame_set(SG_FRAME_PREV, BLACK, 5, GRAY, 30);
	lcd_button_frame_set(SG_FRAME_NEXT, BLACK, 5, GRAY, 30);
	{
		lcd_str(SG_FRAME_PREV[0] + px +  5, SG_FRAME_PREV[1] + py, "PREV",   &Font20, BLACK, WHITE);
		lcd_str(SG_FRAME_NEXT[0] + px +  5, SG_FRAME_NEXT[1] + py, "NEXT",   &Font20, BLACK, WHITE);
	}
}

/** 設定画面の操作入力 **/
bool sg_operation(int *sg_no, axis_t axis_cur) {

	int max = 0;
	switch(*sg_no) {
		case SG_SLEEP:
			max=30;
			break;
		case SG_DRUG_DIR:
		case SG_R_CLICK_DIR:
		case SG_SCROLL_Y_DIR:
		case SG_SCROLL_X_DIR:
			max=DIR_NUM-1;
			break;
		case SG_DRUG_LEN:
		case SG_R_CLICK_LEN:
		case SG_SCROLL_Y_LEN:
		case SG_SCROLL_X_LEN:
			max=115;
			break;
		case SG_TITLE_SPEED:
			max=8;
			break;
		case SG_GYRO:
		case SG_GYRO_SCROLL:
			max=3;
			break;
		case SG_ACC_LIMIT:
			max=20;
			break;
		case SG_ACC_RATE:
			max=10;
			break;
		case SG_TAP_DRAG:
		case SG_VIBRATION:
		case SG_GAME:
		case SG_SCROLL_Y_REV:
		case SG_SCROLL_X_REV:
			max=1;
			break;
	}

	if(is_frame_touch(SG_FRAME_DOWN, axis_cur)) {
		if(*sg_no != SG_EXIT) {
			printf("SG_FRAME_DOWN\n");
			if(g_sg_data[*sg_no] > 0) {
				g_sg_data[*sg_no] --;
			}
		} else {
			// CANCLE
			load_sg_from_flash();	// フラッシュから設定読み取り
			*sg_no = SG_NUM;
		}
		return true;
	} else if(is_frame_touch(SG_FRAME_UP, axis_cur)) {
		if(*sg_no != SG_EXIT) {
			printf("SG_FRAME_UP\n");
			if(g_sg_data[*sg_no] < max) {
				g_sg_data[*sg_no] ++;
			}
		} else {
			// SAVE
			save_sg_to_flash();	// 設定保存
			*sg_no = SG_NUM;
		}
		return true;
	} else if(is_frame_touch(SG_FRAME_NEXT, axis_cur)) {
		printf("SG_FRAME_NEXT\n");
		if(*sg_no < SG_NUM - 1) {
			*sg_no = *sg_no + 1;	// 次の項目へ
		} else {
			*sg_no = 0;		// 最後から最初に戻る
		}
		return true;
	}
	if(is_frame_touch(SG_FRAME_PREV, axis_cur)) {
		printf("SG_FRAME_PREV\n");
		if(*sg_no > 0) {
			*sg_no = *sg_no - 1;	// 前の項目へ
		} else {
			*sg_no = SG_NUM - 1;	// 最初から最後に行く
		}
		return true;
	}
	if(is_frame_touch(SG_FRAME_RESET, axis_cur)) {
		printf("SG_FRAME_RESET\n");
		if(*sg_no == SG_EXIT) {
			init_sg();		// 設定初期化
			*sg_no = SG_NUM;
		}
		return true;
	}
	return false; // 操作なし
}

/** スクリーンセーバー解除 **/
void screenSaverOff() {
	screensaver=g_sg_data[SG_SLEEP] * 50;
	if(plosa->is_sleeping){
	      lcd_set_brightness(plosa->BRIGHTNESS);
	      lcd_sleepoff();
	}
	plosa->is_sleeping=false;
}

/** 起動画面 **/
void start_display() {
	printf("start display start \r\n");
			
	lcd_clr(BLACK);
	lcd_title_set();

	int speed = g_sg_data[SG_TITLE_SPEED];

	if(1 <= speed && speed <= 8) {
		for(int idx=0; idx < 120/speed; idx++) {
			int len = idx * speed;
			lcd_range_line_draw(COLOR_DRAG,     DIR_TOP,    len);
			lcd_range_line_draw(COLOR_R_CLICK,  DIR_BOTTOM, len);
			lcd_range_line_draw(COLOR_SCROLL_X, DIR_LEFT,   len);
			lcd_range_line_draw(COLOR_SCROLL_Y, DIR_RIGHT,  len);

			lcd_display(b0);
		}
	}
	lcd_clr(BLACK);		// 画面クリア
	lcd_display(b0);
	printf("start display end \r\n");
}

/** 設定画面処理ループ **/
void sg_display_loop() {
	start_display();

	printf("sg_display_loop start\r\n");

	axis_t axis_cur;				// 現在座標
	int touch_mode = 0;				// 操作モード
	int sg_no = SG_SLEEP;				// 設定項目番号
	uint32_t last_touch_time = time_us_32();	// 最後に触った時刻
	uint32_t renzoku_time = time_us_32();		// 押しっぱなし判定用

	lcd_clr(BLACK);		// 画面クリア
	lcd_sg_draw(sg_no);

	while(true) {
		// 軌跡を表示
		lcd_circle_guard(axis_cur.x, axis_cur.y, 3, GREEN, 1, false);

		// タッチが行われた場合
		if(flag_touch){
			axis_cur = axis_rotate(); // 画面方向に合わせてX,Y座標を補正して取得
		
			// タッチをしはじめた時
			if( ((time_us_32()-last_touch_time)/MS) > TOUCH_START_MSEC_LIMIT){
				printf("TOUCH START\r\n");
				touch_mode = MODE_TOUCHING;
				renzoku_time = time_us_32();
			} else {
				if( ((time_us_32()-renzoku_time)/MS) > RENZOKU_TOUCH_MSEC_LIMIT){
					touch_mode = MODE_TOUCHING;
				} else {
					touch_mode = MODE_NONE;
				}
			}
			last_touch_time = time_us_32();
			flag_touch = 0;
		} else {
			touch_mode = MODE_NONE;
			renzoku_time = time_us_32();
		}

		// 各種操作
		bool b_op = false; // 操作したかどうかフラグ
		if(touch_mode == MODE_TOUCHING) {
			b_op = sg_operation(&sg_no, axis_cur);
			if(sg_no == SG_NUM) {
				// ループ終了
				break;
			}
			if(sg_no == SG_EXIT) {
				// EXITなら一旦連続押しをやめる
				renzoku_time = time_us_32() + RENZOKU_TOUCH_MSEC_LIMIT;
				touch_mode == MODE_NONE;
			}
		}
		if(b_op) {
			lcd_sg_draw(sg_no);
		}
		lcd_display(b0);
	}

	screenSaverOff();

	start_display();			// 画面クリア
}
		
void scroll_function_inner(bool bY, int move) {
	printf("scroll bY=%d move=%d\n", bY, move);
	if(bY) {
		i2c_data_set(LCDPADKEY_NOCHANGE, 0, 0, 0, move);
	} else {
		i2c_data_set(LCDPADKEY_NOCHANGE, 0, 0, move, 0);
	}
}

/** スクロール処理 **/
scroll_t scroll_function(bool bY, axis_t axis_delta, scroll_t scrt, int16_t lcd_bg_color) {

	int delta = 0;
	if(bY) {
		if(g_sg_data[SG_SCROLL_Y_DIR] == DIR_LEFT || g_sg_data[SG_SCROLL_Y_DIR] == DIR_RIGHT ){
 			delta = -axis_delta.y;
		} else {
 			delta = -axis_delta.x;
		}
		delta = g_sg_data[SG_SCROLL_Y_REV] ? -1 * delta : delta; // 反転
	} else {
	    	if(g_sg_data[SG_SCROLL_X_DIR] == DIR_TOP  || g_sg_data[SG_SCROLL_X_DIR] == DIR_BOTTOM){
			delta = axis_delta.x ;
		} else {
			delta = axis_delta.y ;
		} 
 		delta = g_sg_data[SG_SCROLL_X_REV] ? -1 * delta : delta; // 反転
	}

	if(abs(delta) >= 1) {
		lcd_text_set(3, lcd_bg_color, true, "SCROLL %s:%d", bY ? "Y": "X", delta);

		if(scrt.count % 3 == 0) {
			int move = ceil(scrt.sum / 3);
			scroll_function_inner(bY, move);
			trigger_vibration(60 + abs(scrt.sum) * 3);

			if(abs(scrt.sum) > 0) {
				scrt.last= scrt.sum;
			}
			scrt.sum = 0;
		} else {
			scrt.sum += delta;
		}
		scrt.count++;
	} else {
		// スクロール状態で押しっぱなしの時
		if(scrt.count > 12) {
			int move = scrt.last > 0 ? 1 : -1;
			scroll_function_inner(bY, move);
			trigger_vibration(41);
		}
	}
	return scrt;
}

volatile bool g_flag_click = false; // クリック確定の送信判定フラグ

/** クリック確定の送信処理 **/
int64_t click_commit(alarm_id_t id, void *user_data) {
	if(g_flag_click) {
		i2c_data_set(LCDPADKEY_NONE_CLICK, 0, 0, 0, 0);
		g_flag_click = false;
	} else {
		printf("click_commit cancel\r\n");
	}

    	return 0;
}

/** クリック確定のコールバック設定 **/
void click_commit_timer(int msec) {
	printf("click_commit_timer msec=%d\r\n", msec);
	int *itmp;
	g_flag_click = true;
	set_timer(msec, click_commit, itmp);
}

/** ジャイロ操作 **/
axis_t gyro_function(axis_t axis_gyro_old, uint16_t lcd_bg_color) {
	// ジャイロ操作取得
	QMI8658_read_xyz(acc, gyro, &tim_count);		
	int8_t gyroY = (int8_t)get_acc0();
	int8_t gyroX = (int8_t)get_acc1();

	if(g_sg_data[SG_GYRO] > 0) {
		// ジャイロでポインタ操作
		if(abs(gyroX) > 2 || abs(gyroY) > 2) {
			i2c_data_set(LCDPADKEY_NOCHANGE, gyroX*g_sg_data[SG_GYRO]/5, -gyroY*g_sg_data[SG_GYRO]/5, 0, 0);
		}
	} else if(g_sg_data[SG_GYRO_SCROLL] > 0) {
		// ジャイロでスクロール
		if(abs(gyroX) > 2 || abs(gyroY) > 2) {
			printf("gyro scroll\n");
			i2c_data_set(LCDPADKEY_NOCHANGE, 0, 0, gyroX*g_sg_data[SG_GYRO_SCROLL]/5, gyroY*g_sg_data[SG_GYRO_SCROLL]/5);
		}
	}

	// ジャイロ状態表示
	if(abs(abs(axis_gyro_old.x) - abs(gyroX)) > 0 || abs(abs(axis_gyro_old.y) - abs(gyroY)) > 0 ) {
		screenSaverOff();
		// ジャイロの値を表示
		lcd_text_set(2, lcd_bg_color, false, "GX=%d GY=%d", gyroX, gyroY);
		
		// 古い方を消す	
		lcd_circle_guard  (SCREEN_WIDTH / 2 + axis_gyro_old.x * 2, SCREEN_HEIGHT / 2 - axis_gyro_old.y * 2, 4, lcd_bg_color, 1, true);
		
		// 中心点	
		lcd_circle_guard(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2, 6, WHITE, 1, false);

		if(gyroX == 0 && gyroY ==0) {
			// 中央の場合
			if(g_sg_data[SG_GAME] == 1) {
				trigger_vibration(300);
				center_point_flash(10);
			}
		} else {
			// 今のジャイロ値の場所を描画
			lcd_circle_guard  (SCREEN_WIDTH / 2 + gyroX * 2, SCREEN_HEIGHT / 2 - gyroY * 2, 4, RED, 1, true);
		}
		
		axis_gyro_old.x = gyroX;
		axis_gyro_old.y = gyroY;
	}

	return axis_gyro_old;
}

/** 設定画面呼び出し判定 **/
sg_trigger_t sg_trigger_function(sg_trigger_t sg_trigger, axis_t axis_cur, uint16_t lcd_bg_color) {
	// 設定画面呼び出し操作時間判定
	bool b_sg_trigger_time = (time_us_32()-sg_trigger.time)/MS < TRIGGER_SG_TIME;

	// 設定画面の呼び出し操作状態判定
	if(       sg_trigger.cnt == 0 && axis_cur.x < TRIGGER_SG_LEFT) {
		printf("sg_trigger.cnt=%d\r\n", sg_trigger.cnt);
		sg_trigger.cnt++;
	} else if(sg_trigger.cnt == 1 && axis_cur.x > TRIGGER_SG_RIGHT) {
		printf("sg_trigger.cnt=%d\r\n", sg_trigger.cnt);
		sg_trigger.cnt++;
	} else if(sg_trigger.cnt == 2 && axis_cur.x < TRIGGER_SG_LEFT) {
		printf("sg_trigger.cnt=%d\r\n", sg_trigger.cnt);
		sg_trigger.cnt++;
	} else if(sg_trigger.cnt == 3 && axis_cur.x > TRIGGER_SG_RIGHT) {
		printf("sg_trigger.cnt=%d\r\n", sg_trigger.cnt);
		if(b_sg_trigger_time) {
			lcd_text_set(3, lcd_bg_color, true, "SG START");
			sg_display_loop(); // 設定画面呼び出し
			sg_trigger.cnt=0;
		}
	}

	// 設定画面の呼び出し操作時間切れ
	if(!b_sg_trigger_time) {
		// 設定操作開始状態クリア
		lcd_text_set(3, lcd_bg_color, false,"");
		printf("sg_trigger clear!!\r\n");
		sg_trigger.cnt = 0;
		sg_trigger.time = time_us_32();
	}
	return sg_trigger;
}

/** メイン処理ループ **/
void mouse_display_loop() {
	axis_t axis_cur;				// 現在座標
	axis_t axis_old;				// 一つ前の座標
	axis_t axis_delta;				// 移動量
	axis_t axis_touch;				// タッチ開始時の座標
	axis_t axis_release;				// リリース時の座標

	int touch_mode = 0;				// 操作モード
	int release_cnt = 0;				// タッチを離してる間のカウント値
	uint16_t lcd_bg_color = BLACK;			// LCD背景色
	uint16_t lcd_bg_color_old = BLACK;		// LCD背景色前状態
	uint32_t last_touch_time = time_us_32();	// 最後に触った時刻

	bool b_drag_prestate = false;			// ドラッグ開始前のクリック状態

	sg_trigger_t sg_trigger;
	sg_trigger.time = time_us_32();

	scroll_t scrt = {0};				// スクロール変数

	// ジャイロ用
	axis_t axis_gyro_old;				// 以前の傾き

	printf("loop start !!\r\n");
	
	while(true){
		if(lcd_bg_color != lcd_bg_color_old) {
		        lcd_clr(lcd_bg_color); // 背景色変更
		}
		lcd_bg_color_old = lcd_bg_color;

		// タッチが行われた場合
		if(flag_touch){
			release_cnt++;
			screenSaverOff();	
			axis_cur = axis_rotate(); // 画面方向に合わせてX,Y座標を補正して取得
		
			// 軌跡を表示
			lcd_circle_guard(axis_cur.x, axis_cur.y, 3, GREEN, 1, false);

			// 座標表示
			lcd_text_set(1, lcd_bg_color, true, "X:%03d Y:%03d", axis_cur.x, axis_cur.y);
		
			// タッチをしはじめた時
			if( ((time_us_32()-last_touch_time)/MS) > TOUCH_START_MSEC_LIMIT){
				printf("TOUCH START\r\n");
				touch_mode = MODE_TOUCHING;

				// タップによるドラッグ開始判定
				if(g_sg_data[SG_TAP_DRAG] && (time_us_32() - last_touch_time)/MS < DRAG_START_MSEC && isNearbyPoint(axis_release, axis_cur, 20)) {
					printf("double touch drag prestage\r\n");
					b_drag_prestate = true;
					g_flag_click = false; // クリック確定の送信をキャンセル
				}
		
				// 縦スクロール判定
				if(isRangePress(axis_cur, g_sg_data[SG_SCROLL_Y_DIR], g_sg_data[SG_SCROLL_Y_LEN])) {
					printf("start scroll y\r\n");
					touch_mode = MODE_SCROLL_Y;
				}
				
				// 横スクロール判定
				if(isRangePress(axis_cur, g_sg_data[SG_SCROLL_X_DIR], g_sg_data[SG_SCROLL_X_LEN])) {
					printf("start scroll x\r\n");
					touch_mode = MODE_SCROLL_X;
				}
				
				// タップしてる軌跡を消す処理
				printf("line clear: %08x %d\r\n",last_touch_time,((time_us_32()-last_touch_time)/MS));
				lcd_clr(lcd_bg_color);
				
				release_cnt = 0;
		
				axis_touch = axis_cur;
			} 
				
			// 長押しドラッグ開始判定
			if(isKeepPress(release_cnt, axis_touch, axis_cur, g_sg_data[SG_DRUG_DIR], g_sg_data[SG_DRUG_LEN])) {
				printf("drag start\r\n");
				touch_mode = MODE_DRAG;
			}
		
			// 長押し右クリック判定
			if(isKeepPress(release_cnt, axis_touch, axis_cur, g_sg_data[SG_R_CLICK_DIR], g_sg_data[SG_R_CLICK_LEN])) {
				printf("r click\r\n");
				touch_mode = MODE_R_CLICK;
			}

			// 設定画面呼び出し判定
			sg_trigger = sg_trigger_function(sg_trigger, axis_cur, lcd_bg_color);
		
			// 	
			last_touch_time = time_us_32();
			flag_touch = 0;
		} else {
			if(touch_mode != MODE_TOUCH_RELEASE) {
				if(touch_mode != MODE_NONE) {
					lcd_text_set(3, lcd_bg_color, true,"TOUCH RELEASE");
					touch_mode = MODE_TOUCH_RELEASE;
					axis_old   = axis_0;
	
					// スクロール変数リセット
					scrt.count = 0;
					scrt.sum = 0;
				} else {
					touch_mode = MODE_NONE;
				}
			}

			// スクリーンセーバー
			if(g_sg_data[SG_SLEEP]) {
				screensaver--;
				if(screensaver<=0){
					plosa->is_sleeping=true;
					screensaver=g_sg_data[SG_SLEEP] * 50;
					lcd_set_brightness(0);
					lcd_sleepon();
				}
			}
		} // if(flag_touch) END

		// 各種操作	
		switch(touch_mode) {
			case MODE_TOUCHING:
				lcd_text_set(3, lcd_bg_color, false, "TOUCHING");
				axis_delta = get_axis_delta(axis_cur, axis_old, 1);
				axis_delta = acc_axis_delta(axis_delta);
				i2c_data_set(LCDPADKEY_NOCHANGE, axis_delta.x, axis_delta.y, 0, 0);
				axis_old = axis_cur; 
				break;	
			case MODE_SCROLL_Y:
				if(isRangePress(axis_cur, g_sg_data[SG_SCROLL_Y_DIR], g_sg_data[SG_SCROLL_Y_LEN])) {
					g_flag_click = false; // クリック確定の送信をキャンセル
					axis_delta = get_axis_delta(axis_cur, axis_old, 0.5);
					scrt =  scroll_function(true, axis_delta,  scrt, lcd_bg_color); // スクロール処理
					last_touch_time = time_us_32();	// 最後に触った時刻
				} else {
					touch_mode = MODE_TOUCHING;
				}
				axis_old = axis_cur;
				break;	
			case MODE_SCROLL_X:
				if(isRangePress(axis_cur, g_sg_data[SG_SCROLL_X_DIR], g_sg_data[SG_SCROLL_X_LEN])) {
					g_flag_click = false; // クリック確定の送信をキャンセル
					axis_delta = get_axis_delta(axis_cur, axis_old, 0.5);
					scrt =  scroll_function(false, axis_delta,  scrt, lcd_bg_color); // スクロール処理
					last_touch_time = time_us_32();	// 最後に触った時刻
				} else {
					touch_mode = MODE_TOUCHING;
				}
				axis_old = axis_cur;
				break;	
			case MODE_DRAG:
				lcd_text_set(3, lcd_bg_color, true, "DRAG");
				b_drag_prestate = false;
				
				g_flag_click = false; // クリック確定の送信をキャンセル

				lcd_bg_color = COLOR_DRAG;
				trigger_vibration(150);

				axis_delta = get_axis_delta(axis_cur, axis_old, 0.7);
				i2c_data_set(LCDPADKEY_CLICK_LEFT, axis_delta.x, axis_delta.y, 0, 0);
				axis_old = axis_cur;
				
				touch_mode = MODE_TOUCHING;

				break;	
			case MODE_R_CLICK:
				lcd_text_set(3, lcd_bg_color, true, "R CLICK");
				lcd_bg_color = BLACK;
				
				trigger_vibration(120);
				
				i2c_data_set(LCDPADKEY_CLICK_RIGHT, 0, 0, 0, 0);
				click_commit_timer(DRAG_START_MSEC+10);

				touch_mode = MODE_NONE;
				axis_touch = axis_0;
				axis_old   = axis_0;
				break;	
			case MODE_TOUCH_RELEASE:
				if(release_cnt < SG_CLICK_RELEASE_COUNT_LIMIT) {
					b_drag_prestate = false;
					// 左クリック
					lcd_text_set(3, lcd_bg_color, true, "L CLICK release_cnt=%d", release_cnt);

					lcd_bg_color = BLACK;	
			
					i2c_data_set(LCDPADKEY_CLICK_LEFT, 0, 0, 0, 0);
					click_commit_timer(DRAG_START_MSEC+10);
					release_cnt = SG_CLICK_RELEASE_COUNT_LIMIT;
					axis_old   = axis_0;
				}
				axis_release = axis_cur;
				break;	
			case MODE_NONE:
				break;	
			default:
				printf("axis_old clear 1\n");	
				axis_old   = axis_0;
				break;	
		}
		
		// ダブルタップのドラッグ判定
		if(b_drag_prestate && touch_mode == MODE_TOUCHING && !isNearbyPoint(axis_touch, axis_cur, 1)) {
			// ドラッグ前段階　かつ　タッチ状態で閾値以上離れた場合
			printf("tap drag mode\n");
			touch_mode = MODE_DRAG;
		}

		// ジャイロ操作
		axis_gyro_old =	gyro_function(axis_gyro_old, lcd_bg_color);

		// 画面描画
		lcd_text_draw(lcd_bg_color);
		lcd_display(b0);
	}
}

int main(void) {
	init();			// 初期化処理
	
	load_sg_from_flash();	// フラッシュから設定読み取り

	check_sg();		// フラッシュデータチェック	

	start_display();	// 起動画面

	mouse_display_loop();	// メイン処理ループ
	
	return 0;
}

