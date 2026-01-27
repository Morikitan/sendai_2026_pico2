#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void MainToLineSetup();
void PutDataFromLineToMain();
void LineToMainSetup();
void GetDataFromLineToMain();
void picoPioUartTx_program_putc(unsigned char c, bool even_parity);
unsigned char picoPioUartRx_program_getc(bool even_parity,bool* parity_check);

#ifdef __cplusplus
}
#endif