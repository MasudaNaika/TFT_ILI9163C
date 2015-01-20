#include "TFT_ILI9163C.h"

/**
 * TFT_ILI9163C library for Arduino UNO / LEOPARD
 * 
 * @author Copyright (c) 2014, .S.U.M.O.T.O.Y., coded by Max MC Costa
 * @author modified by masuda, Masuda Naika
 */

//constructors
TFT_ILI9163C::TFT_ILI9163C(uint8_t cspin,uint8_t dcpin,uint8_t rstpin) : Adafruit_GFX(_TFTWIDTH,_TFTHEIGHT){
	_cs   = cspin;
	_dc   = dcpin;
	_rst  = rstpin;
}


TFT_ILI9163C::TFT_ILI9163C(uint8_t cspin,uint8_t dcpin) : Adafruit_GFX(_TFTWIDTH, _TFTHEIGHT) {
  _cs   = cspin;
  _dc   = dcpin;
  _rst  = 0;
}

#if defined(__AVR_MSPIM__)
inline void TFT_ILI9163C::waitSpiFree() {

	while ((UCSRnA & _BV(TXCn)) == 0);
}

inline void TFT_ILI9163C::waitBufferFree() {

	while ((UCSRnA & _BV(UDREn)) == 0);
}

void TFT_ILI9163C::writecommand(uint8_t c){
	
	*dcport &= ~dcpinmask;
	*csport &= ~cspinmask;
	
	UCSRnA |= _BV(TXCn);
	UDRn = c;
	
	waitSpiFree();
	*csport |= cspinmask;
}

void TFT_ILI9163C::writedata(uint8_t c){

	*dcport |= dcpinmask;
	*csport &= ~cspinmask;
	
	UCSRnA |= _BV(TXCn);
	UDRn = c;
	
	waitSpiFree();
	*csport |= cspinmask;
}

void TFT_ILI9163C::writedata16(uint16_t d){
	
	*dcport |= dcpinmask;
	*csport &= ~cspinmask;
	
	UDRn = d >> 8;
	waitBufferFree();
	UCSRnA |= _BV(TXCn);
	UDRn = d & 0xff;
	
	waitSpiFree();
	*csport |= cspinmask;
}

void TFT_ILI9163C::writedata32(uint16_t d1, uint16_t d2){
	
	*dcport |= dcpinmask;
	*csport &= ~cspinmask;
	
	UDRn = d1 >> 8;
	waitBufferFree();
	UDRn = d1 & 0xff;
	waitBufferFree();
	UDRn = d2 >> 8;
	waitBufferFree();
	UCSRnA |= _BV(TXCn);
	UDRn = d2 & 0xff;
	
	waitSpiFree();
	*csport |= cspinmask;
}

void TFT_ILI9163C::writedata16burst(uint16_t d, int32_t len) {
	
	if (len < 0) {
		len = -len;
	}
	
	*dcport |=  dcpinmask;
	*csport &= ~cspinmask;
	
	uint8_t hi = d >> 8;
	uint8_t lo = d & 0xff;
	
	while (len--) {
		waitBufferFree();
		UDRn = hi;
		waitBufferFree();
		UCSRnA |= _BV(TXCn);
		UDRn = lo;
	}
	
	waitSpiFree();
	*csport |= cspinmask;
}

void TFT_ILI9163C::setBitrate(uint32_t n){
	
	uint8_t _ubrrn;
	if (n >= 8000000) {
		_ubrrn = 0;
	} else if (n >= 4000000) {
		_ubrrn = 1;
	} else if (n >= 2000000) {
		_ubrrn = 3;
	} else {
		_ubrrn = 7;
	}
	*csport |= cspinmask;                   // deselect slave
	UCSRnB = 0;								// transmit disable
    UBRRn = 0;                              // must be zero before enabling the transmitter
    UCSRnA = _BV(TXCn);                     // any old transmit now complete
    UCSRnC = _BV(UMSELn0) | _BV(UMSELn1);   // Master SPI mode, SPI mode = 0
    UCSRnB = _BV(TXENn);      				// transmit enable, no TX complete interrupt
    UBRRn = _ubrrn;                         // set bit rate
}

#else
inline void TFT_ILI9163C::waitSpiFree() {

	while((SPSR & _BV(SPIF)) == 0);
}

inline void TFT_ILI9163C::waitBufferFree() {
}

void TFT_ILI9163C::writecommand(uint8_t c){
	
	*dcport &= ~dcpinmask;	// command = low
	*csport &= ~cspinmask;	// select slave = low
	
	SPDR = c;
	waitSpiFree();
	
	*csport |= cspinmask;	// deselect slave = high
}

void TFT_ILI9163C::writedata(uint8_t c){
	
	*dcport |=  dcpinmask;
	*csport &= ~cspinmask;
	
	SPDR = c;
	waitSpiFree();
	
	*csport |= cspinmask;
} 

void TFT_ILI9163C::writedata16(uint16_t d){
	
	*dcport |=  dcpinmask;
	*csport &= ~cspinmask;
	
	SPDR = d >> 8;
	waitSpiFree();
	SPDR = d & 0xff;
	waitSpiFree();
	
	*csport |= cspinmask;
} 

void TFT_ILI9163C::writedata32(uint16_t d1, uint16_t d2){
	
	*dcport |=  dcpinmask;
	*csport &= ~cspinmask;
	
	SPDR = d1 >> 8;
	waitSpiFree();
	SPDR = d1 & 0xff;
	waitSpiFree();
	SPDR = d2 >> 8;
	waitSpiFree();
	SPDR = d2 & 0xff;
	waitSpiFree();
	
	*csport |= cspinmask;
}

void TFT_ILI9163C::writedata16burst(uint16_t d, int32_t len) {
	
	if (len < 0) {
		len = -len;
	}
	
	*dcport |=  dcpinmask;
	*csport &= ~cspinmask;
	
	while (len--) {
		SPDR = d >> 8;
		waitSpiFree();
		SPDR = d & 0xff;
		waitSpiFree();
	}
	
	*csport |= cspinmask;
}

void TFT_ILI9163C::setBitrate(uint32_t n){
	if (n >= 8000000) {
		SPI.setClockDivider(SPI_CLOCK_DIV2);
	} else if (n >= 4000000) {
		SPI.setClockDivider(SPI_CLOCK_DIV4);
	} else if (n >= 2000000) {
		SPI.setClockDivider(SPI_CLOCK_DIV8);
	} else {
		SPI.setClockDivider(SPI_CLOCK_DIV16);
	}
}
#endif


void TFT_ILI9163C::begin(void) {
	
	pinMode(_cs, OUTPUT);
	pinMode(_dc, OUTPUT);
	csport    = portOutputRegister(digitalPinToPort(_cs));
	dcport    = portOutputRegister(digitalPinToPort(_dc));
	cspinmask = digitalPinToBitMask(_cs);
	dcpinmask = digitalPinToBitMask(_dc);
	
// masuda^
#if defined(__AVR_MSPIM__)
	#if defined(__AVR_ATmega32U4__)
		DDRD |= _BV(PIND3) + _BV(PIND5); // mega32u4 PIND3 = TXD, PIND5 = XCK
	#else
//		DDRD |= _BV(PIND1) + _BV(PIND4); // mega328p PIND1 = TXD, PIND4 = XCK
		pinMode(1, OUTPUT);
		pinMode(4, OUTPUT);
	#endif
	// 8 MHz MSPIM, MSB_FIRST, SPI_MODE0
	setBitrate(1000000);
#else
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV4); // 4 MHz (half speed)
    //Due defaults to 4mHz (clock divider setting of 21)
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
#endif

	// toggle RST low to reset; CS low so it'll listen to us
	*csport &= ~cspinmask;


  if (_rst != 0) {
    pinMode(_rst, OUTPUT);
    digitalWrite(_rst, HIGH);
    delay(500);
    digitalWrite(_rst, LOW);
    delay(500);
    digitalWrite(_rst, HIGH);
    delay(500);
  }

/*
7) MY:  1(bottom to top), 0(top to bottom) 	Row Address Order
6) MX:  1(R to L),        0(L to R)        	Column Address Order
5) MV:  1(Exchanged),     0(normal)        	Row/Column exchange
4) ML:  1(bottom to top), 0(top to bottom) 	Vertical Refresh Order
3) RGB: 1(BGR), 		   0(RGB)           	Color Space
2) MH:  1(R to L),        0(L to R)        	Horizontal Refresh Order
1)
0)

     MY, MX, MV, ML,RGB, MH, D1, D0
	 0 | 0 | 0 | 0 | 1 | 0 | 0 | 0	//normal
	 1 | 0 | 0 | 0 | 1 | 0 | 0 | 0	//Y-Mirror
	 0 | 1 | 0 | 0 | 1 | 0 | 0 | 0	//X-Mirror
	 1 | 1 | 0 | 0 | 1 | 0 | 0 | 0	//X-Y-Mirror
	 0 | 0 | 1 | 0 | 1 | 0 | 0 | 0	//X-Y Exchange
	 1 | 0 | 1 | 0 | 1 | 0 | 0 | 0	//X-Y Exchange, Y-Mirror
	 0 | 1 | 1 | 0 | 1 | 0 | 0 | 0	//XY exchange
	 1 | 1 | 1 | 0 | 1 | 0 | 0 | 0
*/
  _Mactrl_Data = 0b00000000;
  _colorspaceData = __COLORSPC;//start with default data;
  chipInit();
}



void TFT_ILI9163C::chipInit() {
	writecommand(CMD_SWRESET);//software reset
	delay(500);
	writecommand(CMD_SLPOUT);//exit sleep
	delay(5);
	writecommand(CMD_PIXFMT);//Set Color Format 16bit   
	writedata(0x05);
	delay(5);
	writecommand(CMD_GAMMASET);//default gamma curve 3
	writedata(0x04);//0x04
	delay(1);
	writecommand(CMD_GAMRSEL);//Enable Gamma adj    
	writedata(0x01); 
	delay(1);
	writecommand(CMD_NORML);
	
	writecommand(CMD_DFUNCTR);
	writedata(0b11111111);//
	writedata(0b00000110);//

	writecommand(CMD_PGAMMAC);//Positive Gamma Correction Setting
	#if defined(__GAMMASET1)
		writedata(0x36);//p1
		writedata(0x29);//p2
		writedata(0x12);//p3
		writedata(0x22);//p4
		writedata(0x1C);//p5
		writedata(0x15);//p6
		writedata(0x42);//p7
		writedata(0xB7);//p8
		writedata(0x2F);//p9
		writedata(0x13);//p10
		writedata(0x12);//p11
		writedata(0x0A);//p12
		writedata(0x11);//p13
		writedata(0x0B);//p14
		writedata(0x06);//p15
	#else
		writedata(0x3F);//p1
		writedata(0x25);//p2
		writedata(0x1C);//p3
		writedata(0x1E);//p4
		writedata(0x20);//p5
		writedata(0x12);//p6
		writedata(0x2A);//p7
		writedata(0x90);//p8
		writedata(0x24);//p9
		writedata(0x11);//p10
		writedata(0x00);//p11
		writedata(0x00);//p12
		writedata(0x00);//p13
		writedata(0x00);//p14
		writedata(0x00);//p15
	#endif

	writecommand(CMD_NGAMMAC);//Negative Gamma Correction Setting
	#if defined(__GAMMASET1)
		writedata(0x09);//p1
		writedata(0x16);//p2
		writedata(0x2D);//p3
		writedata(0x0D);//p4
		writedata(0x13);//p5
		writedata(0x15);//p6
		writedata(0x40);//p7
		writedata(0x48);//p8
		writedata(0x53);//p9
		writedata(0x0C);//p10
		writedata(0x1D);//p11
		writedata(0x25);//p12
		writedata(0x2E);//p13
		writedata(0x34);//p14
		writedata(0x39);//p15
	#else
		writedata(0x20);//p1
		writedata(0x20);//p2
		writedata(0x20);//p3
		writedata(0x20);//p4
		writedata(0x05);//p5
		writedata(0x15);//p6
		writedata(0x00);//p7
		writedata(0xA7);//p8
		writedata(0x3D);//p9
		writedata(0x18);//p10
		writedata(0x25);//p11
		writedata(0x2A);//p12
		writedata(0x2B);//p13
		writedata(0x2B);//p14
		writedata(0x3A);//p15
	#endif

	writecommand(CMD_FRMCTR1);//Frame Rate Control (In normal mode/Full colors)
	writedata(0x08);//0x0C//0x08
	writedata(0x02);//0x14//0x08
	delay(1);
	writecommand(CMD_DINVCTR);//display inversion 
	writedata(0x07);
    delay(1);
	writecommand(CMD_PWCTR1);//Set VRH1[4:0] & VC[2:0] for VCI1 & GVDD   
	writedata(0x0A);//4.30 - 0x0A
	writedata(0x02);//0x05
	delay(1);
	writecommand(CMD_PWCTR2);//Set BT[2:0] for AVDD & VCL & VGH & VGL   
	writedata(0x02);
	delay(1);
	writecommand(CMD_VCOMCTR1);//Set VMH[6:0] & VML[6:0] for VOMH & VCOML   
	writedata(0x50);//0x50
	writedata(99);//0x5b
	delay(1);
	writecommand(CMD_VCOMOFFS);
	writedata(0);//0x40
	delay(1);
  
	writecommand(CMD_CLMADRS);//Set Column Address  
	writedata(0x00); 
	writedata(0X00); 
	writedata(0X00); 
	writedata(_GRAMWIDTH); 
  
	writecommand(CMD_PGEADRS);//Set Page Address  
	writedata(0x00); 
	writedata(0X00); 
	writedata(0X00); 
	writedata(_GRAMHEIGH); 

	colorSpace(_colorspaceData);
	setRotation(0);
	writecommand(CMD_DISPON);//display ON 
	delay(1);
	writecommand(CMD_RAMWR);//Memory Write

	delay(1);
	fillScreen(BLACK);
}

/*
Colorspace selection:
0: RGB
1: GBR
*/
void TFT_ILI9163C::colorSpace(uint8_t cspace) {
	if (cspace < 1){
		bitClear(_Mactrl_Data,3);
	} else {
		bitSet(_Mactrl_Data,3);
	}
}


void TFT_ILI9163C::clearScreen(uint16_t color) {
	homeAddress();
	writedata16burst(color, _GRAMSIZE);
}

void TFT_ILI9163C::homeAddress() {
	setAddrWindow(0x00,0x00,_GRAMWIDTH-1,_GRAMHEIGH-1);
}



void TFT_ILI9163C::setCursor(int16_t x, int16_t y) {
	if (boundaryCheck(x,y)) return;
	setAddrWindow(0x00,0x00,x,y);
	cursor_x = x;
	cursor_y = y;
}



void TFT_ILI9163C::pushColor(uint16_t color) {
	 writedata16(color);
}

void TFT_ILI9163C::drawPixel(int16_t x, int16_t y, uint16_t color) {
	if (boundaryCheck(x,y)) return;
	if ((x < 0) || (y < 0)) return;
	setAddrWindow(x,y,x+1,y+1);
	writedata16(color);
}


void TFT_ILI9163C::drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) {
	// Rudimentary clipping
	if (boundaryCheck(x,y)) return;
	if (((y + h) - 1) >= _height) h = _height-y;
	setAddrWindow(x,y,x,(y+h)-1);
	writedata16burst(color, h);
}

bool TFT_ILI9163C::boundaryCheck(int16_t x,int16_t y){
	if ((x >= _width) || (y >= _height)) return true;
	return false;
}

void TFT_ILI9163C::drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) {
	// Rudimentary clipping
	if (boundaryCheck(x,y)) return;
	if (((x+w) - 1) >= _width)  w = _width-x;
	setAddrWindow(x,y,(x+w)-1,y);
	writedata16burst(color, w);
}

void TFT_ILI9163C::fillScreen(uint16_t color) {
	clearScreen(color);
}

// fill a rectangle
void TFT_ILI9163C::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
	if (boundaryCheck(x,y)) return;
	if (((x + w) - 1) >= _width)  w = _width  - x;
	if (((y + h) - 1) >= _height) h = _height - y;
	setAddrWindow(x,y,(x+w)-1,(y+h)-1);
	
	writedata16burst(color, w * h);
}


// Pass 8-bit (each) R,G,B, get back 16-bit packed color

uint16_t TFT_ILI9163C::Color565(uint8_t r, uint8_t g, uint8_t b) {
	return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}


void TFT_ILI9163C::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {

	writecommand(CMD_CLMADRS); // Column
	if (rotation == 1) {
		writedata32(x0 + __OFFSET, x1 + __OFFSET);
	} else {
		writedata32(x0, x1);
	}

	writecommand(CMD_PGEADRS); // Page
	if (rotation == 0){
		writedata32(y0 + __OFFSET, y1 + __OFFSET);
	} else {
		writedata32(y0, y1);
	}
	
	writecommand(CMD_RAMWR); //Into RAM
}


void TFT_ILI9163C::setRotation(uint8_t m) {
	rotation = m % 4; // can't be higher than 3
	switch (rotation) {
	case 0:
		_Mactrl_Data = 0b00001000;
		_width  = _TFTWIDTH;
		_height = _TFTHEIGHT;//-__OFFSET;
		break;
	case 1:
		_Mactrl_Data = 0b01101000;
		_width  = _TFTHEIGHT;//-__OFFSET;
		_height = _TFTWIDTH;
		break;
	case 2:
		_Mactrl_Data = 0b11001000;
		_width  = _TFTWIDTH;
		_height = _TFTHEIGHT;//-__OFFSET;
		break;
	case 3:
		_Mactrl_Data = 0b10101000;
		_width  = _TFTWIDTH;
		_height = _TFTHEIGHT;//-__OFFSET;
		break;
	}
	colorSpace(_colorspaceData);
	writecommand(CMD_MADCTL);
	writedata(_Mactrl_Data);
}


void TFT_ILI9163C::invertDisplay(boolean i) {
	writecommand(i ? CMD_DINVON : CMD_DINVOF);
}
