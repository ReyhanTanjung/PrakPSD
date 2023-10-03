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
#define FILTER_ORDER 256
#define DUMMY_DATA_LEN 100
#define SAMPLING_FREQ 16000
uint64_t SAMPLING_PERIOD = 1000000 / SAMPLING_FREQ;
const uint32_t TOTAL_DATA_LEN = DATA_LEN + 2*DUMMY_DATA_LEN;
const int32_t OFFSET_ADC = 255*V_OFFSET/ESP_VREF;
// mendefinisikan koefisien filter
float COEFF_B[FILTER_ORDER + 1] = {-0.00019,-0.00016,-0.00012,-0.00009,-0.00006,-0.00004,-0.00002,-0.00000,-0.00000,-0.00000,-0.00002,-0.00004,-0.00008,-0.00012,-0.00018,-0.00024,-0.00031,-0.00039,-0.00047,
-0.00056,-0.00064,-0.00071,-0.00078,-0.00084,-0.00088,-0.00090,-0.00090,-0.00088,-0.00084,-0.00077,-0.00067,-0.00056,-0.00042,-0.00027,-0.00011,0.00006,0.00022,0.00037,0.00051,0.00063,0.00072,0.00077,0.00078,
0.00075,0.00068,0.00056,0.00041,0.00022,-0.00000,-0.00024,-0.00048,-0.00072,-0.00094,-0.00113,-0.00128,-0.00137,-0.00138,-0.00132,-0.00117,-0.00093,-0.00059,-0.00016,0.00035,0.00094,0.00159,0.00228,0.00300,
0.00372,0.00442,0.00507,0.00565,0.00614,0.00652,0.00677,0.00687,0.00683,0.00663,0.00629,0.00582,0.00523,0.00455,0.00380,0.00303,0.00226,0.00155,0.00092,0.00043,0.00011,-0.00000,0.00012,0.00050,0.00115,0.00208,
0.00327,0.00472,0.00639,0.00824,0.01023,0.01229,0.01437,0.01638,0.01826,0.01992,0.02128,0.02228,0.02285,0.02293,0.02247,0.02143,0.01980,0.01757,0.01474,0.01136,0.00746,0.00311,-0.00161,-0.00663,-0.01183,-0.01711,
-0.02235,-0.02744,-0.03226,-0.03670,-0.04065,-0.04402,-0.04674,-0.04872,-0.04994,0.95652,-0.04994,-0.04872,-0.04674,-0.04402,-0.04065,-0.03670,-0.03226,-0.02744,-0.02235,-0.01711,-0.01183,-0.00663,-0.00161,0.00311,
0.00746,0.01136,0.01474,0.01757,0.01980,0.02143,0.02247,0.02293,0.02285,0.02228,0.02128,0.01992,0.01826,0.01638,0.01437,0.01229,0.01023,0.00824,0.00639,0.00472,0.00327,0.00208,0.00115,0.00050,0.00012,-0.00000,0.00011,
0.00043,0.00092,0.00155,0.00226,0.00303,0.00380,0.00455,0.00523,0.00582,0.00629,0.00663,0.00683,0.00687,0.00677,0.00652,0.00614,0.00565,0.00507,0.00442,0.00372,0.00300,0.00228,0.00159,0.00094,0.00035,-0.00016,-0.00059,
-0.00093,-0.00117,-0.00132,-0.00138,-0.00137,-0.00128,-0.00113,-0.00094,-0.00072,-0.00048,-0.00024,-0.00000,0.00022,0.00041,0.00056,0.00068,0.00075,0.00078,0.00077,0.00072,0.00063,0.00051,0.00037,0.00022,0.00006,-0.00011,
-0.00027,-0.00042,-0.00056,-0.00067,-0.00077,-0.00084,-0.00088,-0.00090,-0.00090,-0.00088,-0.00084,-0.00078,-0.00071,-0.00064,-0.00056,-0.00047,-0.00039,-0.00031,-0.00024,-0.00018,-0.00012,-0.00008,-0.00004,-0.00002,-0.00000,-0.00000,-0.00000,-0.00002,-0.00004,-0.00006,-0.00009,-0.00012,-0.00016,-0.00019};
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
// looping yang terjadi adalah perhitungan dan pass data ke 
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