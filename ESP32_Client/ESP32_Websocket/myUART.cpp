#include "myUART.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "myMessage.h"
#include "myDataHandle.h"

static QueueHandle_t uart_queue;

uint8_t uart2_rx_buffer[BUFFER_SIZE] = {0}; // 수신 데이터 버퍼



void uart_receive_task(void)
{
    uart_event_t event;
    // uint8_t uart2_rx_buffer[BUFFER_SIZE] ={0};
  // Serial.println("check here");

   
    if(xQueueReceive(uart_queue, (void *)&event, 10 / portTICK_PERIOD_MS)) {
      switch(event.type) {
          case UART_DATA:
              // 데이터 수신
              uart_read_bytes(UART_NUM, uart2_rx_buffer, event.size, portMAX_DELAY);
              // 여기에서 수신된 데이터 처리
              // Serial.printf("eventsize : %d\n",event.size);
              read_datas_from_TC275(uart2_rx_buffer);
              uart_flush_input(UART_NUM);
              // uart_write_bytes(UART_NUM, "good", 4);
              break;
          case UART_FIFO_OVF:
              // FIFO 오버플로우 처리
              uart_flush_input(UART_NUM);
              xQueueReset(uart_queue);
              break;
          case UART_BUFFER_FULL:
              // 버퍼 풀 처리
              uart_flush_input(UART_NUM);
              xQueueReset(uart_queue);
              break;
          default:
              break;
      }
  
    }

} 

void init_uart2(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // UART 드라이버 설치
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, 1024 * 2, 1024 * 2, 20, &uart_queue, 0));
    
    // UART 파라미터 설정
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    
    // UART 핀 설정
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // UART 이벤트 처리 태스크 생성
    // xTaskCreatePinnedToCore(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL,1);

    Serial.println("Uart2 Initialized");
}


// void uart_event_task(void *pvParameters)
// {
//     uart_event_t event;
//     // uint8_t uart2_rx_buffer[BUFFER_SIZE] ={0};
//   // Serial.println("check here");

//     while(1) {
//       if(xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY)) {
//           switch(event.type) {
//               case UART_DATA:
//                   // 데이터 수신
//                   uart_read_bytes(UART_NUM, uart2_rx_buffer, event.size, portMAX_DELAY);
//                   // 여기에서 수신된 데이터 처리
//                   // Serial.printf("eventsize : %d\n",event.size);
//                   read_datas_from_TC275(uart2_rx_buffer);
//                   uart_write_bytes(UART_NUM, "good", 4);
//                   break;
//               case UART_FIFO_OVF:
//                   // FIFO 오버플로우 처리
//                   uart_flush_input(UART_NUM);
//                   xQueueReset(uart_queue);
//                   break;
//               case UART_BUFFER_FULL:
//                   // 버퍼 풀 처리
//                   uart_flush_input(UART_NUM);
//                   xQueueReset(uart_queue);
//                   break;
//               default:
//                   break;
//           }
//       }
//     }

//     vTaskDelete(NULL);
// } 
