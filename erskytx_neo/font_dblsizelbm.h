const uint8_t font_dblsize[] = {

0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //' '
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
#ifndef BOOT
#ifndef DBL_FONT_SMALL
0x00,0x00,0x00,0x00,0xff,0xff,0x00,0x00,0x00,0x00, //'!'
0x00,0x00,0x00,0x00,0x38,0x38,0x00,0x00,0x00,0x00,

0x00,0x00,0x1f,0x3f,0x00,0x00,0x3f,0x1f,0x00,0x00, //'"'
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

0x30,0x30,0xff,0xff,0x30,0x30,0xff,0xff,0x30,0x30, //'#'
0x03,0x03,0x3f,0x3f,0x03,0x03,0x3f,0x3f,0x03,0x03,

0x30,0x78,0xcc,0xcc,0xff,0xff,0xcc,0xcc,0x8c,0x0c, //'$'
0x0c,0x0c,0x0c,0x0c,0x3f,0x3f,0x0c,0x0c,0x07,0x03,

0x06,0x0f,0x0f,0x86,0xc0,0xe0,0x70,0x38,0x1c,0x0c, //'%'
0x0c,0x0e,0x07,0x03,0x01,0x00,0x18,0x3c,0x3c,0x18,

0x3c,0xfe,0xc7,0xe3,0xf3,0x3b,0x1e,0x0c,0x00,0x00, //'&'
0x0f,0x1f,0x38,0x31,0x33,0x33,0x1e,0x1e,0x33,0x33,

0x00,0x00,0x33,0x3b,0x1f,0x0f,0x00,0x00,0x00,0x00, //'''
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

0x00,0x00,0xf0,0xf8,0x1c,0x0e,0x07,0x03,0x00,0x00, //'('
0x00,0x00,0x03,0x07,0x0e,0x1c,0x38,0x30,0x00,0x00,

0x00,0x00,0x03,0x07,0x0e,0x1c,0xf8,0xf0,0x00,0x00, //')'
0x00,0x00,0x30,0x38,0x1c,0x0e,0x07,0x03,0x00,0x00,

0x30,0x30,0xe0,0xc0,0xfc,0xfc,0xc0,0xe0,0x30,0x30, //'*'
0x03,0x03,0x01,0x00,0x0f,0x0f,0x00,0x01,0x03,0x03,

0xc0,0xc0,0xc0,0xc0,0xfc,0xfc,0xc0,0xc0,0xc0,0xc0, //'+'
0x00,0x00,0x00,0x00,0x0f,0x0f,0x00,0x00,0x00,0x00,
#endif
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //','
0x00,0x00,0x33,0x3b,0x1f,0x0f,0x00,0x00,0x00,0x00,

0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0, //'-'
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //'.'
0x00,0x00,0x18,0x3c,0x3c,0x18,0x00,0x00,0x00,0x00,

0x00,0x00,0x00,0x80,0xc0,0xe0,0x70,0x38,0x1c,0x0c, //'/'
0x0c,0x0e,0x07,0x03,0x01,0x00,0x00,0x00,0x00,0x00,




0x00,0x00,0xfc,0xfe,0x87,0xc3,0xe3,0x77,0xfe,0xfc, //'0'
0x00,0x00,0x0f,0x1f,0x3b,0x31,0x30,0x38,0x1f,0x0f,

0x00,0x00,0x0c,0x0e,0xff,0xff,0x00,0x00,0x00,0x00, //'1'
0x00,0x00,0x30,0x30,0x3f,0x3f,0x30,0x30,0x00,0x00,

0x00,0x00,0x0c,0x0e,0x07,0x83,0xc3,0xe7,0x7e,0x3c, //'2'
0x00,0x00,0x3c,0x3e,0x37,0x33,0x31,0x30,0x30,0x30,

0x00,0x00,0x03,0x03,0xc3,0xc3,0xc3,0xe7,0xfe,0x3c, //'3'
0x00,0x00,0x30,0x30,0x30,0x30,0x30,0x39,0x1f,0x0f,

0x00,0x00,0xc0,0xe0,0x70,0x38,0x1c,0x0e,0xff,0xff, //'4'
0x00,0x00,0x03,0x03,0x03,0x03,0x03,0x03,0x3f,0x3f,

0x00,0x00,0x3f,0x3f,0x33,0x33,0x33,0x73,0xe3,0xc3, //'5'
0x00,0x00,0x0c,0x1c,0x38,0x30,0x30,0x38,0x1f,0x0f,

0x00,0x00,0xfc,0xfe,0xc7,0xc3,0xc3,0xc7,0x8e,0x0c, //'6'
0x00,0x00,0x0f,0x1f,0x38,0x30,0x30,0x39,0x1f,0x0f,

0x00,0x00,0x03,0x03,0xc3,0xe3,0x73,0x3b,0x1f,0x0f, //'7'
0x00,0x00,0x00,0x00,0x3f,0x3f,0x00,0x00,0x00,0x00,

0x00,0x00,0x3c,0xfe,0xe7,0xc3,0xc3,0xe7,0xfe,0x3c, //'8'
0x00,0x00,0x0f,0x1f,0x39,0x30,0x30,0x39,0x1f,0x0f,

0x00,0x00,0x3c,0x7e,0xe7,0xc3,0xc3,0xe7,0xfe,0xfc, //'9'
0x00,0x00,0x00,0x00,0x30,0x38,0x1c,0x0e,0x07,0x03,

0x00,0x00,0x18,0x3c,0x3c,0x18,0x00,0x00,0x00,0x00, //':'
0x00,0x00,0x06,0x0f,0x0f,0x06,0x00,0x00,0x00,0x00,

#ifndef DBL_FONT_SMALL
0x00,0x00,0x18,0x3c,0x3c,0x18,0x00,0x00,0x00,0x00, //';'
0x00,0x00,0x33,0x3b,0x1f,0x0f,0x00,0x00,0x00,0x00,

0x00,0x00,0xc0,0xe0,0xf0,0x38,0x1c,0x0e,0x07,0x03, //'<'
0x00,0x00,0x00,0x01,0x03,0x07,0x0e,0x1c,0x38,0x30,

0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30, //'='
0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,

0x03,0x07,0x0e,0x1c,0x38,0xf0,0xe0,0xc0,0x00,0x00, //'>'
0x30,0x38,0x1c,0x0e,0x07,0x03,0x01,0x00,0x00,0x00,

0x0c,0x0e,0x07,0x03,0x03,0x83,0xc3,0xe7,0x7e,0x3c, //'?'
0x00,0x00,0x00,0x00,0x33,0x33,0x01,0x00,0x00,0x00,




0x00,0x00,0x3c,0x7e,0xe7,0xc3,0xc3,0xe7,0x7e,0x3c, //'@'
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
#endif
0xf0,0xf8,0xdc,0xce,0xc7,0xc7,0xce,0xdc,0xf8,0xf0, //'A'
0x3f,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0x3f,0x3f,
#endif // BOOT

0xff,0xff,0xc3,0xc3,0xc3,0xc3,0xc3,0xe7,0xfe,0x3c, //''
0x3f,0x3f,0x30,0x30,0x30,0x30,0x30,0x39,0x1f,0x0f,

#ifndef BOOT
0xfc,0xfe,0x07,0x03,0x03,0x03,0x03,0x07,0x0e,0x0c, //''
0x0f,0x1f,0x38,0x30,0x30,0x30,0x30,0x38,0x1c,0x0c,

0xff,0xff,0x03,0x03,0x03,0x07,0x0e,0x1c,0xf8,0xf0, //''
0x3f,0x3f,0x30,0x30,0x30,0x38,0x1c,0x0e,0x07,0x03,

0xff,0xff,0xc3,0xc3,0xc3,0xc3,0xc3,0xc3,0x03,0x03, //''
0x3f,0x3f,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,

0xff,0xff,0xc3,0xc3,0xc3,0xc3,0xc3,0xc3,0x03,0x03, //''
0x3f,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

0xfc,0xfe,0x07,0x03,0xc3,0xc3,0xc3,0xc7,0xce,0xcc, //''
0x0f,0x1f,0x38,0x30,0x30,0x30,0x30,0x30,0x3f,0x3f,

0xff,0xff,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xff,0xff, //''
0x3f,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0x3f,0x3f,

0x00,0x00,0x03,0x03,0xff,0xff,0x03,0x03,0x00,0x00, //''
0x00,0x00,0x30,0x30,0x3f,0x3f,0x30,0x30,0x00,0x00,

0x00,0x00,0x00,0x00,0x03,0x03,0xff,0xff,0x03,0x03, //''
0x0c,0x1c,0x38,0x30,0x30,0x38,0x1f,0x0f,0x00,0x00,

0xff,0xff,0xc0,0xe0,0xf0,0x38,0x1c,0x0e,0x07,0x03, //''
0x3f,0x3f,0x00,0x01,0x03,0x07,0x0e,0x1c,0x38,0x30,

0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //''
0x3f,0x3f,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,

0xff,0xff,0x0e,0x1c,0xf8,0xf8,0x1c,0x0e,0xff,0xff, //''
0x3f,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0x3f,0x3f,

0xff,0xff,0x38,0x70,0xe0,0xc0,0x80,0x00,0xff,0xff, //''
0x3f,0x3f,0x00,0x00,0x00,0x01,0x03,0x03,0x3f,0x3f,

0xfc,0xfe,0x07,0x03,0x03,0x03,0x03,0x07,0xfe,0xfc, //''
0x0f,0x1f,0x38,0x30,0x30,0x30,0x30,0x38,0x1f,0x0f,




0xff,0xff,0xc3,0xc3,0xc3,0xc3,0xc3,0xe7,0x7e,0x3c, //'P'
0x3f,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

0xfc,0xfe,0x07,0x03,0x03,0x03,0x03,0x07,0xfe,0xfc, //''
0x0f,0x1f,0x38,0x30,0x33,0x37,0x1e,0x1c,0x3f,0x33,

0xff,0xff,0xc3,0xc3,0xc3,0xc3,0xc3,0xe7,0x7e,0x3c, //''
0x3f,0x3f,0x00,0x00,0x03,0x07,0x0e,0x1c,0x38,0x30,
#endif // BOOT

0x3c,0x7e,0xe7,0xc3,0xc3,0xc3,0xc3,0xc7,0x8e,0x0c, //'S'
0x0c,0x1c,0x38,0x30,0x30,0x30,0x30,0x39,0x1f,0x0f,

#ifndef BOOT
0x03,0x03,0x03,0x03,0xff,0xff,0x03,0x03,0x03,0x03, //''
0x00,0x00,0x00,0x00,0x3f,0x3f,0x00,0x00,0x00,0x00,
#endif // BOOT

0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff, //'U'
0x0f,0x1f,0x38,0x30,0x30,0x30,0x30,0x38,0x1f,0x0f,

#ifndef BOOT
0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff, //''
0x03,0x07,0x0e,0x1c,0x38,0x38,0x1c,0x0e,0x07,0x03,

0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff, //''
0x0f,0x1f,0x38,0x38,0x1f,0x1f,0x38,0x38,0x1f,0x0f,

0x0f,0x1f,0x38,0xf0,0xe0,0xe0,0xf0,0x38,0x1f,0x0f, //''
0x3c,0x3e,0x07,0x03,0x01,0x01,0x03,0x07,0x3e,0x3c,

0x3f,0x7f,0xe0,0xc0,0x80,0x80,0xc0,0xe0,0x7f,0x3f, //''
0x00,0x00,0x00,0x01,0x3f,0x3f,0x01,0x00,0x00,0x00,

0x03,0x03,0x03,0x83,0xc3,0xe3,0x73,0x3b,0x1f,0x0f, //'Z'
0x3c,0x3e,0x37,0x33,0x31,0x30,0x30,0x30,0x30,0x30,

#ifndef DBL_FONT_SMALL
0x00,0x00,0xff,0xff,0x03,0x03,0x03,0x03,0x00,0x00, //'['
0x00,0x00,0x3f,0x3f,0x30,0x30,0x30,0x30,0x00,0x00,

0x0c,0x1c,0x38,0x70,0xe0,0xc0,0x80,0x00,0x00,0x00, //'\'
0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x07,0x0e,0x0c,

0x00,0x00,0x03,0x03,0x03,0x03,0xff,0xff,0x00,0x00, //']'
0x00,0x00,0x30,0x30,0x30,0x30,0x3f,0x3f,0x00,0x00,

0x30,0x38,0x1c,0x0e,0x07,0x07,0x0e,0x1c,0x38,0x30, //'^'
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
#endif

0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //'_'
0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,



#ifndef DBL_FONT_SMALL
0x00,0x00,0x03,0x07,0x0e,0x1c,0x38,0x30,0x00,0x00, //'`'
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
#endif
0x00,0x00,0x30,0x30,0x30,0x30,0x30,0x70,0xe0,0xc0, //'a'
0x0c,0x1e,0x3f,0x33,0x33,0x33,0x33,0x33,0x3f,0x3f,

0xff,0xff,0xc0,0xe0,0x70,0x30,0x30,0x70,0xe0,0xc0, //''
0x3f,0x3f,0x30,0x30,0x30,0x30,0x30,0x38,0x1f,0x0f,

0xc0,0xe0,0x70,0x30,0x30,0x30,0x30,0x30,0x00,0x00, //''
0x0f,0x1f,0x38,0x30,0x30,0x30,0x30,0x38,0x1c,0x0c,

0xc0,0xe0,0x70,0x30,0x30,0x70,0xe0,0xc0,0xff,0xff, //''
0x0f,0x1f,0x38,0x30,0x30,0x30,0x30,0x30,0x3f,0x3f,

0xc0,0xe0,0x70,0x30,0x30,0x30,0x30,0x70,0xe0,0xc0, //''
0x0f,0x1f,0x3b,0x33,0x33,0x33,0x33,0x33,0x03,0x03,

0xc0,0xc0,0xfc,0xfe,0xc7,0xc3,0x03,0x07,0x0e,0x0c, //''
0x00,0x00,0x3f,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,

0xf0,0xf8,0x9c,0x0c,0x0c,0x0c,0x0c,0x0c,0xfc,0xfc, //''
0x00,0x01,0x33,0x33,0x33,0x33,0x33,0x3b,0x1f,0x0f,

0xff,0xff,0xc0,0xe0,0x70,0x30,0x30,0x70,0xe0,0xc0, //''
0x3f,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0x3f,0x3f,

0x00,0x00,0x30,0x30,0xf3,0xf3,0x00,0x00,0x00,0x00, //''
0x00,0x00,0x30,0x30,0x3f,0x3f,0x30,0x30,0x00,0x00,

0x00,0x00,0x00,0x00,0x30,0x30,0xf3,0xf3,0x00,0x00, //''
0x0c,0x1c,0x38,0x30,0x30,0x38,0x1f,0x0f,0x00,0x00,

0xff,0xff,0x00,0x80,0xc0,0xe0,0x70,0x30,0x00,0x00, //''
0x3f,0x3f,0x03,0x07,0x0f,0x1c,0x38,0x30,0x00,0x00,

0x00,0x00,0x03,0x03,0xff,0xff,0x00,0x00,0x00,0x00, //''
0x00,0x00,0x30,0x30,0x3f,0x3f,0x30,0x30,0x00,0x00,

0xf0,0xf0,0x30,0x70,0xe0,0xe0,0x70,0x70,0xe0,0xc0, //''
0x3f,0x3f,0x00,0x00,0x03,0x03,0x00,0x00,0x3f,0x3f,

0xf0,0xf0,0xc0,0xe0,0x70,0x30,0x30,0x70,0xe0,0xc0, //''
0x3f,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0x3f,0x3f,

0xc0,0xe0,0x70,0x30,0x30,0x30,0x30,0x70,0xe0,0xc0, //'o'
0x0f,0x1f,0x38,0x30,0x30,0x30,0x30,0x38,0x1f,0x0f,




0xf0,0xf0,0x30,0x30,0x30,0x30,0x30,0xf0,0xe0,0xc0, //'p'
0x3f,0x3f,0x03,0x03,0x03,0x03,0x03,0x03,0x01,0x00,

0xc0,0xe0,0xf0,0x30,0x30,0x30,0xe0,0xc0,0xf0,0xf0, //''
0x00,0x01,0x03,0x03,0x03,0x03,0x03,0x03,0x3f,0x3f,

0xf0,0xf0,0xc0,0xe0,0x70,0x30,0x30,0x70,0xe0,0xc0, //''
0x3f,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

0xc0,0xe0,0xf0,0x30,0x30,0x30,0x30,0x30,0x00,0x00, //''
0x30,0x31,0x33,0x33,0x33,0x33,0x33,0x3f,0x1e,0x0c,

0x30,0x30,0xff,0xff,0x30,0x30,0x00,0x00,0x00,0x00, //''
0x00,0x00,0x0f,0x1f,0x38,0x30,0x30,0x38,0x1c,0x0c,

0xf0,0xf0,0x00,0x00,0x00,0x00,0x00,0x00,0xf0,0xf0, //''
0x0f,0x1f,0x38,0x30,0x30,0x38,0x1c,0x0c,0x3f,0x3f,

0xf0,0xf0,0x00,0x00,0x00,0x00,0x00,0x00,0xf0,0xf0, //''
0x03,0x07,0x0e,0x1c,0x38,0x38,0x1c,0x0e,0x07,0x03,

0xf0,0xf0,0x00,0x00,0x00,0x00,0x00,0x00,0xf0,0xf0, //''
0x0f,0x1f,0x38,0x38,0x1c,0x1c,0x38,0x38,0x1f,0x0f,

0x30,0x70,0xe0,0xc0,0x80,0x80,0xc0,0xe0,0x70,0x30, //''
0x30,0x38,0x1c,0x0f,0x07,0x07,0x0f,0x1c,0x38,0x30,

0xf0,0xf0,0x80,0x00,0x00,0x00,0x00,0x00,0xf0,0xf0, //''
0x00,0x01,0x33,0x33,0x33,0x33,0x33,0x3b,0x1f,0x0f,

0x30,0x30,0x30,0x30,0x30,0xb0,0xf0,0xf0,0x70,0x30, //'z'
0x30,0x38,0x3c,0x3e,0x37,0x33,0x31,0x30,0x30,0x30,

#ifndef DBL_FONT_SMALL
0x00,0x00,0xc0,0xe0,0x3c,0x3e,0x07,0x03,0x00,0x00, //'{'
0x00,0x00,0x00,0x01,0x0f,0x1f,0x38,0x30,0x00,0x00,

0x00,0x00,0x00,0x00,0xff,0xff,0x00,0x00,0x00,0x00, //'|'
0x00,0x00,0x00,0x00,0x3f,0x3f,0x00,0x00,0x00,0x00,

0x00,0x00,0x03,0x07,0x3e,0x3c,0xe0,0xc0,0x00,0x00, //'}'
0x00,0x00,0x30,0x38,0x1f,0x0f,0x01,0x00,0x00,0x00,

0xc0,0xc0,0xc0,0xc0,0x0c,0x1c,0xf8,0xf0,0xe0,0xc0, //'->'
0x00,0x00,0x00,0x00,0x0c,0x0e,0x07,0x03,0x01,0x00,

0xc0,0xe0,0xf0,0xf8,0x1c,0x0c,0xc0,0xc0,0xc0,0xc0, //'<-'
0x00,0x01,0x03,0x07,0x0e,0x0c,0x00,0x00,0x00,0x00,
#endif
#endif // BOOT
//0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x1f,0x3f,0x00,0x00,0x3f,0x1f,0x00,0x00,0x30,0x30,0xff,0xff,0x30,0x30,0xff,0xff,0x30,0x30,0x30,0x78,0xcc,0xcc,0xff,0xff,0xcc,0xcc,0x8c,0x0c,0x06,0x0f,0x0f,0x86,0xc0,0xe0,0x70,0x38,0x1c,0x0c,0x3c,0xfe,0xc7,0xe3,0xf3,0x3b,0x1e,0x0c,0x00,0x00,0x00,0x00,0x33,0x3b,0x1f,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0xf0,0xf8,0x1c,0x0e,0x07,0x03,0x00,0x00,0x00,0x00,0x03,0x07,0x0e,0x1c,0xf8,0xf0,0x00,0x00,0x30,0x30,0xe0,0xc0,0xfc,0xfc,0xc0,0xe0,0x30,0x30,0xc0,0xc0,0xc0,0xc0,0xfc,0xfc,0xc0,0xc0,0xc0,0xc0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xc0,0xe0,0x70,0x38,0x1c,0x0c,
//0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x38,0x38,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x03,0x3f,0x3f,0x03,0x03,0x3f,0x3f,0x03,0x03,0x0c,0x0c,0x0c,0x0c,0x3f,0x3f,0x0c,0x0c,0x07,0x03,0x0c,0x0e,0x07,0x03,0x01,0x00,0x18,0x3c,0x3c,0x18,0x0f,0x1f,0x38,0x31,0x33,0x33,0x1e,0x1e,0x33,0x33,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x07,0x0e,0x1c,0x38,0x30,0x00,0x00,0x00,0x00,0x30,0x38,0x1c,0x0e,0x07,0x03,0x00,0x00,0x03,0x03,0x01,0x00,0x0f,0x0f,0x00,0x01,0x03,0x03,0x00,0x00,0x00,0x00,0x0f,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x33,0x3b,0x1f,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x3c,0x3c,0x18,0x00,0x00,0x00,0x00,0x0c,0x0e,0x07,0x03,0x01,0x00,0x00,0x00,0x00,0x00,
//0x00,0x00,0xfc,0xfe,0x87,0xc3,0xe3,0x77,0xfe,0xfc,0x00,0x00,0x0c,0x0e,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x0c,0x0e,0x07,0x83,0xc3,0xe7,0x7e,0x3c,0x00,0x00,0x03,0x03,0xc3,0xc3,0xc3,0xe7,0xfe,0x3c,0x00,0x00,0xc0,0xe0,0x70,0x38,0x1c,0x0e,0xff,0xff,0x00,0x00,0x3f,0x3f,0x33,0x33,0x33,0x73,0xe3,0xc3,0x00,0x00,0xfc,0xfe,0xc7,0xc3,0xc3,0xc7,0x8e,0x0c,0x00,0x00,0x03,0x03,0xc3,0xe3,0x73,0x3b,0x1f,0x0f,0x00,0x00,0x3c,0xfe,0xe7,0xc3,0xc3,0xe7,0xfe,0x3c,0x00,0x00,0x3c,0x7e,0xe7,0xc3,0xc3,0xe7,0xfe,0xfc,0x00,0x00,0x18,0x3c,0x3c,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x3c,0x3c,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0xe0,0xf0,0x38,0x1c,0x0e,0x07,0x03,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x03,0x07,0x0e,0x1c,0x38,0xf0,0xe0,0xc0,0x00,0x00,0x0c,0x0e,0x07,0x03,0x03,0x83,0xc3,0xe7,0x7e,0x3c,
//0x00,0x00,0x0f,0x1f,0x3b,0x31,0x30,0x38,0x1f,0x0f,0x00,0x00,0x30,0x30,0x3f,0x3f,0x30,0x30,0x00,0x00,0x00,0x00,0x3c,0x3e,0x37,0x33,0x31,0x30,0x30,0x30,0x00,0x00,0x30,0x30,0x30,0x30,0x30,0x39,0x1f,0x0f,0x00,0x00,0x03,0x03,0x03,0x03,0x03,0x03,0x3f,0x3f,0x00,0x00,0x0c,0x1c,0x38,0x30,0x30,0x38,0x1f,0x0f,0x00,0x00,0x0f,0x1f,0x38,0x30,0x30,0x39,0x1f,0x0f,0x00,0x00,0x00,0x00,0x3f,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0x0f,0x1f,0x39,0x30,0x30,0x39,0x1f,0x0f,0x00,0x00,0x00,0x00,0x30,0x38,0x1c,0x0e,0x07,0x03,0x00,0x00,0x06,0x0f,0x0f,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x33,0x3b,0x1f,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x07,0x0e,0x1c,0x38,0x30,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x30,0x38,0x1c,0x0e,0x07,0x03,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x33,0x33,0x01,0x00,0x00,0x00,
//0x00,0x00,0x3c,0x7e,0xe7,0xc3,0xc3,0xe7,0x7e,0x3c,0xf0,0xf8,0xdc,0xce,0xc7,0xc7,0xce,0xdc,0xf8,0xf0,0xff,0xff,0xc3,0xc3,0xc3,0xc3,0xc3,0xe7,0xfe,0x3c,0xfc,0xfe,0x07,0x03,0x03,0x03,0x03,0x07,0x0e,0x0c,0xff,0xff,0x03,0x03,0x03,0x07,0x0e,0x1c,0xf8,0xf0,0xff,0xff,0xc3,0xc3,0xc3,0xc3,0xc3,0xc3,0x03,0x03,0xff,0xff,0xc3,0xc3,0xc3,0xc3,0xc3,0xc3,0x03,0x03,0xfc,0xfe,0x07,0x03,0xc3,0xc3,0xc3,0xc7,0xce,0xcc,0xff,0xff,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xff,0xff,0x00,0x00,0x03,0x03,0xff,0xff,0x03,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x03,0xff,0xff,0x03,0x03,0xff,0xff,0xc0,0xe0,0xf0,0x38,0x1c,0x0e,0x07,0x03,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0x0e,0x1c,0xf8,0xf8,0x1c,0x0e,0xff,0xff,0xff,0xff,0x38,0x70,0xe0,0xc0,0x80,0x00,0xff,0xff,0xfc,0xfe,0x07,0x03,0x03,0x03,0x03,0x07,0xfe,0xfc,
//0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3f,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0x3f,0x3f,0x3f,0x3f,0x30,0x30,0x30,0x30,0x30,0x39,0x1f,0x0f,0x0f,0x1f,0x38,0x30,0x30,0x30,0x30,0x38,0x1c,0x0c,0x3f,0x3f,0x30,0x30,0x30,0x38,0x1c,0x0e,0x07,0x03,0x3f,0x3f,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x3f,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0f,0x1f,0x38,0x30,0x30,0x30,0x30,0x30,0x3f,0x3f,0x3f,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0x3f,0x3f,0x00,0x00,0x30,0x30,0x3f,0x3f,0x30,0x30,0x00,0x00,0x0c,0x1c,0x38,0x30,0x30,0x38,0x1f,0x0f,0x00,0x00,0x3f,0x3f,0x00,0x01,0x03,0x07,0x0e,0x1c,0x38,0x30,0x3f,0x3f,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x3f,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0x3f,0x3f,0x3f,0x3f,0x00,0x00,0x00,0x01,0x03,0x03,0x3f,0x3f,0x0f,0x1f,0x38,0x30,0x30,0x30,0x30,0x38,0x1f,0x0f,
//0xff,0xff,0xc3,0xc3,0xc3,0xc3,0xc3,0xe7,0x7e,0x3c,0xfc,0xfe,0x07,0x03,0x03,0x03,0x03,0x07,0xfe,0xfc,0xff,0xff,0xc3,0xc3,0xc3,0xc3,0xc3,0xe7,0x7e,0x3c,0x3c,0x7e,0xe7,0xc3,0xc3,0xc3,0xc3,0xc7,0x8e,0x0c,0x03,0x03,0x03,0x03,0xff,0xff,0x03,0x03,0x03,0x03,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0x0f,0x1f,0x38,0xf0,0xe0,0xe0,0xf0,0x38,0x1f,0x0f,0x3f,0x7f,0xe0,0xc0,0x80,0x80,0xc0,0xe0,0x7f,0x3f,0x03,0x03,0x03,0x83,0xc3,0xe3,0x73,0x3b,0x1f,0x0f,0x00,0x00,0xff,0xff,0x03,0x03,0x03,0x03,0x00,0x00,0x0c,0x1c,0x38,0x70,0xe0,0xc0,0x80,0x00,0x00,0x00,0x00,0x00,0x03,0x03,0x03,0x03,0xff,0xff,0x00,0x00,0x30,0x38,0x1c,0x0e,0x07,0x07,0x0e,0x1c,0x38,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
//0x3f,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0f,0x1f,0x38,0x30,0x33,0x37,0x1e,0x1c,0x3f,0x33,0x3f,0x3f,0x00,0x00,0x03,0x07,0x0e,0x1c,0x38,0x30,0x0c,0x1c,0x38,0x30,0x30,0x30,0x30,0x39,0x1f,0x0f,0x00,0x00,0x00,0x00,0x3f,0x3f,0x00,0x00,0x00,0x00,0x0f,0x1f,0x38,0x30,0x30,0x30,0x30,0x38,0x1f,0x0f,0x03,0x07,0x0e,0x1c,0x38,0x38,0x1c,0x0e,0x07,0x03,0x0f,0x1f,0x38,0x38,0x1f,0x1f,0x38,0x38,0x1f,0x0f,0x3c,0x3e,0x07,0x03,0x01,0x01,0x03,0x07,0x3e,0x3c,0x00,0x00,0x00,0x01,0x3f,0x3f,0x01,0x00,0x00,0x00,0x3c,0x3e,0x37,0x33,0x31,0x30,0x30,0x30,0x30,0x30,0x00,0x00,0x3f,0x3f,0x30,0x30,0x30,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x07,0x0e,0x0c,0x00,0x00,0x30,0x30,0x30,0x30,0x3f,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,
//0x00,0x00,0x03,0x07,0x0e,0x1c,0x38,0x30,0x00,0x00,0x00,0x00,0x30,0x30,0x30,0x30,0x30,0x70,0xe0,0xc0,0xff,0xff,0xc0,0xe0,0x70,0x30,0x30,0x70,0xe0,0xc0,0xc0,0xe0,0x70,0x30,0x30,0x30,0x30,0x30,0x00,0x00,0xc0,0xe0,0x70,0x30,0x30,0x70,0xe0,0xc0,0xff,0xff,0xc0,0xe0,0x70,0x30,0x30,0x30,0x30,0x70,0xe0,0xc0,0xc0,0xc0,0xfc,0xfe,0xc7,0xc3,0x03,0x07,0x0e,0x0c,0xf0,0xf8,0x9c,0x0c,0x0c,0x0c,0x0c,0x0c,0xfc,0xfc,0xff,0xff,0xc0,0xe0,0x70,0x30,0x30,0x70,0xe0,0xc0,0x00,0x00,0x30,0x30,0xf3,0xf3,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x30,0xf3,0xf3,0x00,0x00,0xff,0xff,0x00,0x80,0xc0,0xe0,0x70,0x30,0x00,0x00,0x00,0x00,0x03,0x03,0xff,0xff,0x00,0x00,0x00,0x00,0xf0,0xf0,0x30,0x70,0xe0,0xe0,0x70,0x70,0xe0,0xc0,0xf0,0xf0,0xc0,0xe0,0x70,0x30,0x30,0x70,0xe0,0xc0,0xc0,0xe0,0x70,0x30,0x30,0x30,0x30,0x70,0xe0,0xc0,
//0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0c,0x1e,0x3f,0x33,0x33,0x33,0x33,0x33,0x3f,0x3f,0x3f,0x3f,0x30,0x30,0x30,0x30,0x30,0x38,0x1f,0x0f,0x0f,0x1f,0x38,0x30,0x30,0x30,0x30,0x38,0x1c,0x0c,0x0f,0x1f,0x38,0x30,0x30,0x30,0x30,0x30,0x3f,0x3f,0x0f,0x1f,0x3b,0x33,0x33,0x33,0x33,0x33,0x03,0x03,0x00,0x00,0x3f,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x33,0x33,0x33,0x33,0x33,0x3b,0x1f,0x0f,0x3f,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0x3f,0x3f,0x00,0x00,0x30,0x30,0x3f,0x3f,0x30,0x30,0x00,0x00,0x0c,0x1c,0x38,0x30,0x30,0x38,0x1f,0x0f,0x00,0x00,0x3f,0x3f,0x03,0x07,0x0f,0x1c,0x38,0x30,0x00,0x00,0x00,0x00,0x30,0x30,0x3f,0x3f,0x30,0x30,0x00,0x00,0x3f,0x3f,0x00,0x00,0x03,0x03,0x00,0x00,0x3f,0x3f,0x3f,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0x3f,0x3f,0x0f,0x1f,0x38,0x30,0x30,0x30,0x30,0x38,0x1f,0x0f,
//0xf0,0xf0,0x30,0x30,0x30,0x30,0x30,0xf0,0xe0,0xc0,0xc0,0xe0,0xf0,0x30,0x30,0x30,0xe0,0xc0,0xf0,0xf0,0xf0,0xf0,0xc0,0xe0,0x70,0x30,0x30,0x70,0xe0,0xc0,0xc0,0xe0,0xf0,0x30,0x30,0x30,0x30,0x30,0x00,0x00,0x30,0x30,0xff,0xff,0x30,0x30,0x00,0x00,0x00,0x00,0xf0,0xf0,0x00,0x00,0x00,0x00,0x00,0x00,0xf0,0xf0,0xf0,0xf0,0x00,0x00,0x00,0x00,0x00,0x00,0xf0,0xf0,0xf0,0xf0,0x00,0x00,0x00,0x00,0x00,0x00,0xf0,0xf0,0x30,0x70,0xe0,0xc0,0x80,0x80,0xc0,0xe0,0x70,0x30,0xf0,0xf0,0x80,0x00,0x00,0x00,0x00,0x00,0xf0,0xf0,0x30,0x30,0x30,0x30,0x30,0xb0,0xf0,0xf0,0x70,0x30,0x00,0x00,0xc0,0xe0,0x3c,0x3e,0x07,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x07,0x3e,0x3c,0xe0,0xc0,0x00,0x00,0xc0,0xc0,0xc0,0xc0,0x0c,0x1c,0xf8,0xf0,0xe0,0xc0,0xc0,0xe0,0xf0,0xf8,0x1c,0x0c,0xc0,0xc0,0xc0,0xc0,
//0x3f,0x3f,0x03,0x03,0x03,0x03,0x03,0x03,0x01,0x00,0x00,0x01,0x03,0x03,0x03,0x03,0x03,0x03,0x3f,0x3f,0x3f,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x31,0x33,0x33,0x33,0x33,0x33,0x3f,0x1e,0x0c,0x00,0x00,0x0f,0x1f,0x38,0x30,0x30,0x38,0x1c,0x0c,0x0f,0x1f,0x38,0x30,0x30,0x38,0x1c,0x0c,0x3f,0x3f,0x03,0x07,0x0e,0x1c,0x38,0x38,0x1c,0x0e,0x07,0x03,0x0f,0x1f,0x38,0x38,0x1c,0x1c,0x38,0x38,0x1f,0x0f,0x30,0x38,0x1c,0x0f,0x07,0x07,0x0f,0x1c,0x38,0x30,0x00,0x01,0x33,0x33,0x33,0x33,0x33,0x3b,0x1f,0x0f,0x30,0x38,0x3c,0x3e,0x37,0x33,0x31,0x30,0x30,0x30,0x00,0x00,0x00,0x01,0x0f,0x1f,0x38,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x3f,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x38,0x1f,0x0f,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0c,0x0e,0x07,0x03,0x01,0x00,0x00,0x01,0x03,0x07,0x0e,0x0c,0x00,0x00,0x00,0x00,
};