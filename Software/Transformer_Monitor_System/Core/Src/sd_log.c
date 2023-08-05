#include "fatfs.h"
#include "main.h"
#include "stdio.h"
#include "sd_log.h"
#include "string.h"

uint32_t Write_TFCard(char* filename,char* str_zt)
{
	  FIL fil;
	  uint32_t byteswritten=0;
		//printf("\r\n ****** 文件系统 ******\r\n\r\n");
 
    /*-1- 挂载文件系统*/
	  retSD = f_mount(&SDFatFS, "", 0);
    if(retSD)
    {
        printf(" mount error : %d \r\n",retSD);
        Error_Handler();
    }
//    else
//        printf(" mount sucess!!! \r\n");
     
    /*-2-创建新的文件并写入数据*/
    retSD = f_open(&fil, filename, FA_OPEN_ALWAYS | FA_WRITE);		//打开文件，权限包括创建、写（如果没有该文件，会创建该文件）
    if(retSD)															//返回值不为0（出现问题）
        printf(" open file error : %d\r\n",retSD);						//打印问题代码
//    else
//        printf(" open file sucess!!! \r\n");
	
      /*-3- 偏移指针到末尾处*/	   
//	  printf(" file size: %d \r\n",(int)f_size(&fil));
	  f_lseek(&fil,f_size(&fil));
	
    /*-4- 在txt文件尾续写数据*/	
//	  printf("size of str_zt=%d\n",strlen(str_zt));
	  retSD = f_write(&fil, str_zt, strlen(str_zt), (void *)&byteswritten);	//在文件内写入wtext内的内容	

		if(retSD)															//返回值不为0（出现问题）
					printf(" write file error : %d\r\n",retSD);						//打印问题代码
//			else
//			{
//					printf(" write file sucess!!! \r\n");
//					printf(" write Data : %s\r\n",str_zt);							//打印写入的内容
//			}
     
    /*-5- 关闭txt文件*/
    retSD = f_close(&fil);												//关闭该文件
    if(retSD)															//返回值不为0（出现问题）
        printf(" close error : %d\r\n",retSD);							//打印问题代码
//    else
//        printf(" close sucess!!! \r\n");
		return byteswritten;
}
