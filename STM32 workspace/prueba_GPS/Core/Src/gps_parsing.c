/*
 * gps_parsing.c
 *
 *  Created on: Aug 5, 2024
 *      Author: alber_7dxjh2i
 */
#include "gps_parsing.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>


// Procesado del texto, identificación por líneas
void process_gps_data(char *uart_buffer, GPGGA_Data *data) {
    char *line;
    char *next_line = uart_buffer;
    char line_buffer[120];  // Asumiendo que ninguna línea excede los 120 caracteres

    while ((line = strtok_r(next_line, "\r\n", &next_line))) {
        if (strstr(line, "$GPGGA") == line) {  // Verificar si la línea contiene GPGGA
        	snprintf(line_buffer, sizeof(line_buffer), "GPGGA Data received: %s", line);
        	parse_gpgga(line, data);
            print(line_buffer);
        } else if (strstr(line, "$GPRMC") == line) {  // Verificar si la línea contiene GPRMC
            snprintf(line_buffer, sizeof(line_buffer), "GPRMC Data received: %s", line);
            print(line_buffer);
        }else if (strstr(line, "$GPGSA") == line) {  // Verificar si la línea contiene GPRMC
            snprintf(line_buffer, sizeof(line_buffer), "GPGSA Data received: %s", line);
            print(line_buffer);
        }else if (strstr(line, "$GPGSV") == line) {  // Verificar si la línea contiene GPRMC
            snprintf(line_buffer, sizeof(line_buffer), "GPGSV Data received: %s", line);
            print(line_buffer);
        } else {
        	snprintf(line_buffer, sizeof(line_buffer), "Data not identified: %s", line);
        	print(line_buffer);
        }
    }
}

// Función para parsear la trama GPGGA
void parse_gpgga(char *gpgga_sentence, GPGGA_Data *data) {
    // "$GPGGA,202530.00,5109.0262,N,11401.8407,W,5,40,0.5,1097.36,M,-17.00,M,18,TSTR*61"
    // "$GPGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f,%c,%f,%c"

    // Utilizamos sscanf para extraer los campos de la trama
    sscanf(gpgga_sentence, "$GPGGA,%[^,],%[^,],%c,%[^,],%c,%d,%d,%[^,],%[^,],%c,%[^,],%c",
           data->time, data->latitude, &data->lat_direction, data->longitude,
           &data->lon_direction, &data->fix_quality, &data->satellites,
           &data->hdop, &data->altitude, &data->altitude_units,
           &data->geoid_height, &data->geoid_units);

    // Se puede poner los siguiente para que lo muestre por terminal

    print(". . .Parsing . . .");
    char print_buffer[120];
    snprintf(print_buffer, sizeof(print_buffer), "Time: %s", data->time);
    print(print_buffer);
    snprintf(print_buffer, sizeof(print_buffer), "Latitude: %s %c", data->latitude, data->lat_direction);
    print(print_buffer);
    snprintf(print_buffer, sizeof(print_buffer), "Longitude: %s %c", data->longitude, data->lon_direction);
    print(print_buffer);
    snprintf(print_buffer, sizeof(print_buffer), "Fix Quality: %d", data->fix_quality);
    print(print_buffer);
    snprintf(print_buffer, sizeof(print_buffer), "Satellites: %d", data->satellites);
    print(print_buffer);
    snprintf(print_buffer, sizeof(print_buffer), "HDOP: %s", data->hdop);
    print(print_buffer);
    snprintf(print_buffer, sizeof(print_buffer), "Altitude: %s %c", data->altitude, data->altitude_units);
    print(print_buffer);
    snprintf(print_buffer, sizeof(print_buffer), "Geoid Height: %s %c", data->geoid_height, data->geoid_units);
    print(print_buffer);
    print(". . . Finish parsing . . . ");

}



