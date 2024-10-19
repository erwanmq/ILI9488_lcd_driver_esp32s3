#pragma once

struct Position{
    int x;
    int y;
};
struct Size{
    unsigned int width;
    unsigned int height;
};

enum ID{
    ANY = -1,
    BTN_NUMPAD_1,
    BTN_NUMPAD_2,
    BTN_NUMPAD_3,
    BTN_NUMPAD_4,
    BTN_NUMPAD_5,
    BTN_NUMPAD_6,
    BTN_NUMPAD_7,
    BTN_NUMPAD_8,
    BTN_NUMPAD_9,
    BTN_NUMPAD_0,
    BTN_NUMPAD_OK,
    BTN_NUMPAD_LEFT,
    BTN_NUMPAD_RIGHT,
    COUNT,
}; 


enum Click{
    ONE_CLICK,
    DOUBLE_CLICK,
    RELEASE,
    NONE
};
