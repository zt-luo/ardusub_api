/**
 * @file main.c
 * @author Zongtong Luo (luozongtong123@163.com)
 * @brief 
 * @version 0.1
 * @date 2018-10-16
 * 
 * @copyright Copyright (c) 2018
 * 
 */

#include "../include/ardusub_api.h"

/**
 * @brief 
 * 
 * @param parameter_size 
 * @param parameter 
 * @return int 
 */
int main(int argc, char *argv[])
{
    ardusub_api_init();
    ardusub_api_run();  // none return Fun

    return 0;
}
