#ifndef __DATA_SAMPLE_H
#define __DATA_SAMPLE_H

#ifdef __cplusplus
 extern "C" {
#endif

 
void ADS1274_Config(void);
void ADS1274_run(void);
void ADS1274_tcp_send_data(void);

#ifdef __cplusplus
 }
#endif
	 
#endif /* __SPI_FLASH_H */

