// penggunaan library
#include <Adafruit_MCP3008.h>
// definisi pin interkoneksi kit
#define LED_PIN 17
#define STR_BTN 21
#define ADC_ESP 36
#define DAC_1 26
#define DAC_2 25
#define SPI_MISO 19
#define SPI_MOSI 23
#define SPI_SCLK 18
#define SPI_CS 5
// definisi parameter
#define DATA_LEN 2000
#define MCP_VREF 4.75 // Volt DC
#define ESP_VREF 3.30 // Volt DC
#define V_OFFSET 1.60 // Volt DC
#define FILTER_ORDER 32
#define DUMMY_DATA_LEN 100
#define SAMPLING_FREQ 16000
uint64_t SAMPLING_PERIOD = 1000000 / SAMPLING_FREQ;
const uint32_t TOTAL_DATA_LEN = DATA_LEN + 2*DUMMY_DATA_LEN;
const int32_t OFFSET_ADC = 255*V_OFFSET/ESP_VREF;
// mendefinisikan koefisien filter
float COEFF_B[FILTER_ORDER + 1] = {-0.0009,-0.0010,-0.0014,-0.0019,-0.0026,
-0.0034,-0.0044,-0.0055,-0.0066,-0.0078,-0.0089,-0.0099,-0.0108,-0.0115,
-0.0121,-0.0124,0.9884,-0.0124,-0.0121,-0.0115,-0.0108,-0.0099,-0.0089,-0.0078,
-0.0066,-0.0055,-0.0044,-0.0034,-0.0026,-0.0019,-0.0014,-0.0010,-0.0009};
// pembuatan objek adc dari MCP3008
Adafruit_MCP3008 adc;
// pembuatan objek timer
hw_timer_t *My_timer = NULL;
// deklarasi variabel
byte getDataFlag = 0;
byte calcDataFlag = 0;
// mendefinisikan data untuk filter dan hasilnya
int32_t BUFFER[TOTAL_DATA_LEN];
int32_t UNFILTERED_DATA[TOTAL_DATA_LEN];
int32_t FILTERED_DATA[TOTAL_DATA_LEN];
uint32_t counter = DUMMY_DATA_LEN;
// setiap SAMPLING_PERIOD, melakukan interupsi
void IRAM_ATTR onTimer(){
if(getDataFlag){
UNFILTERED_DATA[counter] = ((adc.readADC(0) >> 
2)/ESP_VREF)*MCP_VREF;
BUFFER[counter] = UNFILTERED_DATA[counter] - OFFSET_ADC;
counter = counter + 1;
if(counter >= TOTAL_DATA_LEN){
getDataFlag = 0;
counter = DUMMY_DATA_LEN;
}
}else{
if(!calcDataFlag){
if(counter < DATA_LEN + DUMMY_DATA_LEN){
dacWrite(DAC_1, FILTERED_DATA[counter]);
dacWrite(DAC_2, UNFILTERED_DATA[counter]);
}else{
dacWrite(DAC_1, OFFSET_ADC);
dacWrite(DAC_2, OFFSET_ADC);
if(counter >= (DATA_LEN << 1) + DUMMY_DATA_LEN){
counter = DUMMY_DATA_LEN;
}
}
counter = counter + 1;
}
}
}
// ketika tombol ditekan, akan dilakukan get DATA_LEN data; flag set ke 1
void getData(){
getDataFlag = 1;
calcDataFlag = 1;
counter = 0;
}
// fungsi untuk melakukan filter
int32_t filter_fir(int32_t counter_){
float temp = 0;
int32_t int_temp;
for(uint32_t i = 0; i < FILTER_ORDER; i++){
temp = temp + BUFFER[counter_ - i] * COEFF_B[i];
}
int_temp = (int32_t) temp;
return int_temp;
}
void setup() {
// set mode untuk setiap pin
pinMode(ADC_ESP, INPUT);
pinMode(STR_BTN, INPUT_PULLUP);
pinMode(LED_PIN, OUTPUT);
// menggunakan interrupt external untuk tombol
attachInterrupt(digitalPinToInterrupt(STR_BTN), getData, FALLING);
// inisiasi MCP3008
adc.begin(SPI_SCLK, SPI_MOSI, SPI_MISO, SPI_CS);
// menyalakan led indikator
digitalWrite(LED_PIN, HIGH);
// set timer untuk sampling
My_timer = timerBegin(0, 80, true);
timerAttachInterrupt(My_timer, &onTimer, true);
timerAlarmWrite(My_timer, SAMPLING_PERIOD, true);
timerAlarmEnable(My_timer);
// inisialisasi data terfilter
for(uint32_t i = 0; i < TOTAL_DATA_LEN; i++){
FILTERED_DATA[i] = 128;
}
}
void loop() {
// looping yang terjadi adalah perhitungan dan pass data ke osiloskop
// pass data dilakukan dengan interrupt pada DAC_1 dan DAC_2
if(!getDataFlag && calcDataFlag){
FILTERED_DATA[counter] = filter_fir(counter);
FILTERED_DATA[counter] = FILTERED_DATA[counter] + OFFSET_ADC;
counter = counter + 1;
if(counter >= DUMMY_DATA_LEN + DATA_LEN){
calcDataFlag = 0;
counter = 0;
}
}
}