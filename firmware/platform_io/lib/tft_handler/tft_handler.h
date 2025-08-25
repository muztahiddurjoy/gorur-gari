#ifndef TFT_HANDLER_H
#define TFT_HANDLER_H

#include <TFT_eSPI.h>

void tftSetup();
void tftUpdate(long encoder_count, int button_state, float velocity = 0, float steering_angle = 0);
void tftShowMessage(const String &message, uint16_t color = TFT_YELLOW);
void tftClear();

#endif // TFT_HANDLER_H