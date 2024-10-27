/*
 * gps_parsing.h
 *
 *  Created on: Aug 5, 2024
 *      Author: alber_7dxjh2i
 */

#ifndef INC_GPS_PARSING_H_
#define INC_GPS_PARSING_H_

// Estructura para almacenar los datos GPGGA
// Hay datos que representan floats, pero para parsearlos hace falta poner algo de una flag
// que no consigo encontrar. Con esto lo guarda como un string y sí funciona.
typedef struct {
    char time[11]; 			// float
    char latitude[10];		// float
    char lat_direction;		// char
    char longitude[11];		// float
    char lon_direction;		// char
    int fix_quality;		// int
    int satellites;			// int
    char hdop[8];			// float
    char altitude[8];		// float
    char altitude_units;	// char
    char geoid_height[8];	// float
    char geoid_units;		// char
} GPGGA_Data;

void process_gps_data(char* uart_buffer, GPGGA_Data *data);
void parse_gpgga(char *gpgga_sentence, GPGGA_Data *data);



#endif /* INC_GPS_PARSING_H_ */
