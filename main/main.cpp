#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <driver/i2c.h>

#include "bq27441.h"

static const char* TAG = "MAIN";

#define I2C_SCL_IO				22	//19               /*!< gpio number for I2C master clock */
#define I2C_SDA_IO				21	//18               /*!< gpio number for I2C master data  */
#define I2C_FREQ_HZ				400000           /*!< I2C master clock frequency */
#define I2C_PORT_NUM			I2C_NUM_0        /*!< I2C port number for master dev */
#define I2C_TX_BUF_DISABLE  	0                /*!< I2C master do not need buffer */
#define I2C_RX_BUF_DISABLE  	0                /*!< I2C master do not need buffer */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */

#define BQ27441_I2C_ADDRESS 0x55

extern "C" {
  void app_main(void);
};

static esp_err_t i2c_master_init(void)
{
  i2c_config_t conf = {};
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = (gpio_num_t)I2C_SDA_IO;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = (gpio_num_t)I2C_SCL_IO;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = I2C_FREQ_HZ;
  i2c_param_config(I2C_PORT_NUM, &conf);
  return i2c_driver_install(I2C_PORT_NUM, conf.mode, I2C_RX_BUF_DISABLE, I2C_TX_BUF_DISABLE, 0);
}

void app_main(void)
{
  ESP_LOGI(TAG,"MAIN ENTRY");

  i2c_master_init();
  i2c_set_timeout(I2C_PORT_NUM,0xFFFFF);

  //xTaskCreatePinnedToCore(task_fuel_gauge, "fuel_gauge", 2048, (void* ) 0, 20, NULL,1);

  begin(I2C_PORT_NUM);

  setCapacity(2000);

  while(1)
  {
    vTaskDelay(1000/portTICK_PERIOD_MS);
    unsigned int _soc = soc(FILTERED);  // Read state-of-charge (%)
    unsigned int _volts = voltage(); // Read battery voltage (mV)
    int _current = current(AVG); // Read average current (mA)
    unsigned int _fullCapacity = capacity(FULL); // Read full capacity (mAh)
    unsigned int _capacity = capacity(REMAIN); // Read remaining capacity (mAh)
    int _power = power(); // Read average power draw (mW)
    int _health = soh(PERCENT); // Read state-of-health (%)

    ESP_LOGI(TAG,"Battery SOC %i%% VOLTS %imV CURRENT %imA CAPACITY %i / %imAh",_soc,_volts,_current,_capacity,_fullCapacity);
    ESP_LOGI(TAG,"Battery HEALTH %i%% POWER %imW",_health,_power);
  }
}
