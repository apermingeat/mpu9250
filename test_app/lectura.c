/* @file   testebbchar.c
 * @author Derek Molloy
 * @date   7 April 2015
 * @version 0.1
 * @brief  A Linux user space program that communicates with the ebbchar.c LKM. It passes a
 * string to the LKM and reads the response from the LKM. For this example to work the device
 * must be called /dev/ebbchar.
 * @see http://www.derekmolloy.ie/ for a full description and follow-up descriptions.
 */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#define MPU9250_MAX_DATA_SIZE_16BITS	10 /* x,y,z for acc, gyro, mag + temp*/
#define BUFFER_LENGTH 32               ///< The buffer length (crude but fine)
static int16_t receive[MPU9250_MAX_DATA_SIZE_16BITS];     ///< The receive buffer from the LKM


int main(){
   int ret, fd,cant_lectura;

   printf("Abriendo el dispositivo.\n");
   fd = open("/dev/mi_dispositivo", O_RDWR);             // Open the device with read/write access
   if (fd < 0){
      printf("Failed to open the device...");
      return -1;
   }

   while(1)
   {
   	ret = read(fd, receive, MPU9250_MAX_DATA_SIZE_16BITS*2);        // Read the response from the LKM
   	if (ret < 0){
      		printf("Failed to read the message from the device.");
      		close(fd);
      		return -1;
   	}
	printf("ACCEL x: %d, y: %d, z: %d  --  GYRO x: %d, y: %d, z: %d -- MAG x: %d, y: %d, z: %d, temp %d \n",receive[0],receive[1],receive[2],receive[3],receive[4],receive[5],receive[6],receive[7],receive[8],receive[9],receive[10]);
	sleep(1);
   }
   
//   printf("The received message is: [%s]\n", receive);
   printf("End of the program\n");
   close(fd);
   return 0;
}

