#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <driver/i2c.h>

#include "bq27441.h"

static const char* TAG = "MAIN";


extern "C" {
  void app_main(void);
};

static esp_err_t i2c_master_init(void)
{
  int i2c_master_port = 0;
  i2c_config_t conf = {};
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = (gpio_num_t)21;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = (gpio_num_t)22;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = 10*1000; //100khz
  i2c_param_config(i2c_master_port, &conf);
  return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}


void app_main(void)
{
  ESP_LOGI(TAG,"MAIN ENTRY");

  i2c_master_init();

  bool ret = begin();
  //uint16_t cap = capacity(REMAIN);

  setCapacity(2000);
  ESP_LOGI(TAG,"%i",ret);
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

    ESP_LOGI(TAG,"Battery SOC %i VOLTS %i CURRENT %i CAPACITY %i / %i",_soc,_volts,_current,_capacity,_fullCapacity);
  }
}
