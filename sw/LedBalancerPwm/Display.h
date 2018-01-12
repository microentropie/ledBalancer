#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#define QtRows 4
#define QtColums 20

#ifndef byte
typedef unsigned char byte;
#endif //byte

void display_init(const char *PROGRAM);
void DisplayIfChanged(byte row, String &newValues);
void DisplayIfChanged(byte row, const char *pNewValues);

#endif //_DISPLAY_H_
