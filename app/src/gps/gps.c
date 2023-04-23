#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>

#include <minmea/minmea.h>

#include "gps.h"

LOG_MODULE_REGISTER(gps);

static char rx_buf_gps_msg_uart[MINMEA_MAX_LENGTH];
static int rx_buf_gps_msg_uart_pos = 0;
static struct gps_data current_gps_data = {};

int parse_nmea(const char *line)
{
    switch (minmea_sentence_id(line, false))
    {
    case MINMEA_SENTENCE_RMC:
    {
        struct minmea_sentence_rmc frame;
        if (minmea_parse_rmc(&frame, line))
        {
            // LOG_INF("$xxRMC: raw coordinates and speed: (%d/%d,%d/%d) %d/%d",
            //         frame.latitude.value, frame.latitude.scale,
            //         frame.longitude.value, frame.longitude.scale,
            //         frame.speed.value, frame.speed.scale);
            // LOG_INF("$xxRMC fixed-point coordinates and speed scaled to three decimal places: (%d,%d) %d",
            //         minmea_rescale(&frame.latitude, 1000),
            //         minmea_rescale(&frame.longitude, 1000),
            //         minmea_rescale(&frame.speed, 1000));
            // LOG_INF("$xxRMC floating point degree coordinates and speed: (%f,%f) %f",
            //         minmea_tocoord(&frame.latitude),
            //         minmea_tocoord(&frame.longitude),
            //         minmea_tofloat(&frame.speed));

            current_gps_data.lat = minmea_tocoord(&frame.latitude);
            current_gps_data.lng = minmea_tocoord(&frame.longitude);
            current_gps_data.speed = minmea_tofloat(&frame.speed);
        }
        else
        {
            // LOG_INF("$xxRMC sentence is not parsed");
        }
    }
    break;

        // case MINMEA_SENTENCE_GGA:
        // {
        //     struct minmea_sentence_gga frame;
        //     if (minmea_parse_gga(&frame, line))
        //     {
        //         LOG_INF("$xxGGA: fix quality: %d", frame.fix_quality);
        //     }
        //     else
        //     {
        //         LOG_INF("$xxGGA sentence is not parsed");
        //     }
        // }
        // break;

        // case MINMEA_SENTENCE_GST:
        // {
        //     struct minmea_sentence_gst frame;
        //     if (minmea_parse_gst(&frame, line))
        //     {
        //         LOG_INF("$xxGST: raw latitude,longitude and altitude error deviation: (%d/%d,%d/%d,%d/%d)",
        //                 frame.latitude_error_deviation.value, frame.latitude_error_deviation.scale,
        //                 frame.longitude_error_deviation.value, frame.longitude_error_deviation.scale,
        //                 frame.altitude_error_deviation.value, frame.altitude_error_deviation.scale);
        //         LOG_INF("$xxGST fixed point latitude,longitude and altitude error deviation"
        //                 " scaled to one decimal place: (%d,%d,%d)",
        //                 minmea_rescale(&frame.latitude_error_deviation, 10),
        //                 minmea_rescale(&frame.longitude_error_deviation, 10),
        //                 minmea_rescale(&frame.altitude_error_deviation, 10));
        //         LOG_INF("$xxGST floating point degree latitude, longitude and altitude error deviation: (%f,%f,%f)",
        //                 minmea_tofloat(&frame.latitude_error_deviation),
        //                 minmea_tofloat(&frame.longitude_error_deviation),
        //                 minmea_tofloat(&frame.altitude_error_deviation));
        //     }
        //     else
        //     {
        //         LOG_INF("$xxGST sentence is not parsed");
        //     }
        // }
        // break;

        // case MINMEA_SENTENCE_GSV:
        // {
        //     struct minmea_sentence_gsv frame;
        //     if (minmea_parse_gsv(&frame, line))
        //     {
        //         LOG_INF("$xxGSV: message %d of %d", frame.msg_nr, frame.total_msgs);
        //         LOG_INF("$xxGSV: sattelites in view: %d", frame.total_sats);
        //         for (int i = 0; i < 4; i++)
        //             LOG_INF("$xxGSV: sat nr %d, elevation: %d, azimuth: %d, snr: %d dbm",
        //                     frame.sats[i].nr,
        //                     frame.sats[i].elevation,
        //                     frame.sats[i].azimuth,
        //                     frame.sats[i].snr);
        //     }
        //     else
        //     {
        //         LOG_INF("$xxGSV sentence is not parsed");
        //     }
        // }
        // break;

        // case MINMEA_SENTENCE_VTG:
        // {
        //     struct minmea_sentence_vtg frame;
        //     if (minmea_parse_vtg(&frame, line))
        //     {
        //         LOG_INF("$xxVTG: true track degrees = %f",
        //                 minmea_tofloat(&frame.true_track_degrees));
        //         LOG_INF("        magnetic track degrees = %f",
        //                 minmea_tofloat(&frame.magnetic_track_degrees));
        //         LOG_INF("        speed knots = %f",
        //                 minmea_tofloat(&frame.speed_knots));
        //         LOG_INF("        speed kph = %f",
        //                 minmea_tofloat(&frame.speed_kph));
        //     }
        //     else
        //     {
        //         LOG_INF("$xxVTG sentence is not parsed");
        //     }
        // }
        // break;

        // case MINMEA_SENTENCE_ZDA:
        // {
        //     struct minmea_sentence_zda frame;
        //     if (minmea_parse_zda(&frame, line))
        //     {
        //         LOG_INF("$xxZDA: %d:%d:%d %02d.%02d.%d UTC%+03d:%02d",
        //                 frame.time.hours,
        //                 frame.time.minutes,
        //                 frame.time.seconds,
        //                 frame.date.day,
        //                 frame.date.month,
        //                 frame.date.year,
        //                 frame.hour_offset,
        //                 frame.minute_offset);
        //     }
        //     else
        //     {
        //         LOG_INF("$xxZDA sentence is not parsed");
        //     }
        // }
        // break;

        // case MINMEA_INVALID:
        // {
        //     LOG_INF("$xxxxx sentence is not valid");
        // }
        // break;

    default:
    {
        // LOG_INF("$xxxxx sentence is not parsed");
    }
    break;
    }

    return 0;
}

int gps_get_data(struct gps_data *data)
{
    data->lat = current_gps_data.lat;
    data->lng = current_gps_data.lng;
    data->speed = current_gps_data.speed;
    return 0;
}

// int gps_run()
// {
//     char rx_buf_gps_msg_uart[MINMEA_MAX_LENGTH];
//     int rx_buf_gps_msg_uart_pos = 0;
//     uint8_t c;

//     while (uart_poll_in(gps_dev, &c) == 0)
//     {
//         if (c == '\r')
//         {
//             // Skip
//         }
//         else if (c == '\n' && rx_buf_gps_msg_uart_pos > 0)
//         {
//             /* terminate string */
//             rx_buf_gps_msg_uart[rx_buf_gps_msg_uart_pos] = '\0';

//             LOG_INF("GPS: %s", rx_buf_gps_msg_uart);
//             parse_nmea((const char *)&rx_buf_gps_msg_uart);
//             return 0;
//         }
//         else if (rx_buf_gps_msg_uart_pos < (sizeof(rx_buf_gps_msg_uart) - 1))
//         {
//             rx_buf_gps_msg_uart[rx_buf_gps_msg_uart_pos++] = c;
//             LOG_INF("GPS: %s", c);
//         }
//         /* else: characters beyond buffer size are dropped */
//     }
// }

void gps_uart_cb(const struct device *dev, void *user_data)
{
    uint8_t c;

    if (!uart_irq_update(dev))
    {
        return;
    }

    while (uart_irq_rx_ready(dev))
    {

        uart_fifo_read(dev, &c, 1);

        if (rx_buf_gps_msg_uart_pos > 1 && (c == '\n' && rx_buf_gps_msg_uart[rx_buf_gps_msg_uart_pos - 1] == '\r'))
        {
            /* terminate string */
            rx_buf_gps_msg_uart[rx_buf_gps_msg_uart_pos] = '\n';
            rx_buf_gps_msg_uart[rx_buf_gps_msg_uart_pos + 1] = '\0';

            /* if queue is full, message is silently dropped */
            // k_msgq_put(&lora_uart_msgq, &rx_buf_gps_msg_uart, K_NO_WAIT);

            /* reset the buffer (it was copied to the msgq) */
            rx_buf_gps_msg_uart_pos = 0;

            // LOG_INF("Answer: %s", rx_buf_gps_msg_uart);
            parse_nmea(rx_buf_gps_msg_uart);
        }
        else if (rx_buf_gps_msg_uart_pos < (sizeof(rx_buf_gps_msg_uart) - 1))
        {
            rx_buf_gps_msg_uart[rx_buf_gps_msg_uart_pos++] = c;
        }
        /* else: characters beyond buffer size are dropped */
    }
}

int gps_init(const struct device *dev)
{
    if (!device_is_ready(dev))
    {
        LOG_ERR("gps sensor: device not ready.");
        return -ENODEV;
    }

    /* configure interrupt and callback to receive data */
    uart_irq_callback_user_data_set(dev, gps_uart_cb, NULL);
    uart_irq_rx_enable(dev);

    return 0;
}