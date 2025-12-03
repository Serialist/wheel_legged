/*    ==============================================================================    
ÎÄ¼þ×÷ÓÃ£º´ËÎÄ¼þÎªflash¶ÁÐ´Ïà¹Øº¯ÊýµÄÊµÏÖÎÄ¼þ£¬Ö÷Òª°üº¬ÁËflash²Á³ý¡¢Ð´¡¢¶ÁµÈÏà¹Øº¯ÊýµÄÊµÏÖ¡£
ÎÄ¼þËµÃ÷£º´ËÎÄ¼þÔÚ2025Èü¼¾Ã»ÓÐÊ¹ÓÃ
      ==============================================================================*/
#include "bsp_flash.h"
#include "main.h"
#include "string.h"
/**
  * @brief          get the sector number of flash
  * @param[in]      address: flash address
  * @retval         sector number
  */
/**
  * @brief          ï¿½ï¿½È¡flashï¿½ï¿½sectorï¿½ï¿½
  * @param[in]      address: flash ï¿½ï¿½Ö·
  * @retval         sectorï¿½ï¿½
  */
static uint32_t ger_sector(uint32_t address);

/**
  * @brief          erase flash
  * @param[in]      address: flash address
  * @param[in]      len: page num
  * @retval         none
  */
/**
  * @brief          ï¿½ï¿½ï¿½ï¿½flash
  * @param[in]      address: flash ï¿½ï¿½Ö·
  * @param[in]      len: Ò³ï¿½ï¿½ï¿½ï¿½
  * @retval         none
  */
void flash_erase_address(uint32_t address, uint16_t len)
{
    FLASH_EraseInitTypeDef flash_erase;
    uint32_t error;

    flash_erase.Sector = ger_sector(address);
    flash_erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    flash_erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    flash_erase.NbSectors = len;

    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&flash_erase, &error);
    HAL_FLASH_Lock();
}

/**
  * @brief          write data to one page of flash
  * @param[in]      start_address: flash address
  * @param[in]      buf: data point
  * @param[in]      len: data num
  * @retval         success 0, fail -1
  */
/**
  * @brief          ï¿½ï¿½Ò»Ò³flashÐ´ï¿½ï¿½ï¿½ï¿½
  * @param[in]      start_address: flash ï¿½ï¿½Ö·
  * @param[in]      buf: ï¿½ï¿½ï¿½ï¿½Ö¸ï¿½ï¿½
  * @param[in]      len: ï¿½ï¿½ï¿½Ý³ï¿½ï¿½ï¿½
  * @retval         success 0, fail -1
  */
int8_t flash_write_single_address(uint32_t start_address, uint32_t *buf, uint32_t len)
{
    static uint32_t uw_address;
    static uint32_t end_address;
    static uint32_t *data_buf;
    static uint32_t data_len;

    HAL_FLASH_Unlock();

    uw_address = start_address;
    end_address = get_next_flash_address(start_address);
    data_buf = buf;
    data_len = 0;

    while (uw_address <= end_address)
    {

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,uw_address, *data_buf) == HAL_OK)
        {
            uw_address += 4;
            data_buf++;
            data_len++;
            if (data_len == len)
            {
                break;
            }
        }
        else
        {
            HAL_FLASH_Lock();
            return -1;
        }
    }

    HAL_FLASH_Lock();
    return 0;
}

/**
  * @brief          write data to some pages of flash
  * @param[in]      start_address: flash start address
  * @param[in]      end_address: flash end address
  * @param[in]      buf: data point
  * @param[in]      len: data num
  * @retval         success 0, fail -1
  */
/**
  * @brief          ï¿½ï¿½ï¿½ï¿½Ò³flashÐ´ï¿½ï¿½ï¿½ï¿½
  * @param[in]      start_address: flash ï¿½ï¿½Ê¼ï¿½ï¿½Ö·
  * @param[in]      end_address: flash ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö·
  * @param[in]      buf: ï¿½ï¿½ï¿½ï¿½Ö¸ï¿½ï¿½
  * @param[in]      len: ï¿½ï¿½ï¿½Ý³ï¿½ï¿½ï¿½
  * @retval         success 0, fail -1
  */
int8_t flash_write_muli_address(uint32_t start_address, uint32_t end_address, uint32_t *buf, uint32_t len)
{
    uint32_t uw_address = 0;
    uint32_t *data_buf;
    uint32_t data_len;

    HAL_FLASH_Unlock();

    uw_address = start_address;
    data_buf = buf;
    data_len = 0;
    while (uw_address <= end_address)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,uw_address, *data_buf) == HAL_OK)
        {
            uw_address += 4;
            data_buf++;
            data_len++;
            if (data_len == len)
            {
                break;
            }
        }
        else
        {
            HAL_FLASH_Lock();
            return -1;
        }
    }

    HAL_FLASH_Lock(); 
    return 0;
}

/**
  * @brief          read data for flash
  * @param[in]      address: flash address
  * @param[out]     buf: data point
  * @param[in]      len: data num
  * @retval         none
  */
/**
  * @brief          ï¿½ï¿½flashï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
  * @param[in]      start_address: flash ï¿½ï¿½Ö·
  * @param[out]     buf: ï¿½ï¿½ï¿½ï¿½Ö¸ï¿½ï¿½
  * @param[in]      len: ï¿½ï¿½ï¿½Ý³ï¿½ï¿½ï¿½
  * @retval         none
  */
void flash_read(uint32_t address, uint32_t *buf, uint32_t len)
{
    memcpy(buf, (void*)address, len *4);
}


/**
  * @brief          get the sector number of flash
  * @param[in]      address: flash address
  * @retval         sector number
  */
/**
  * @brief          ï¿½ï¿½È¡flashï¿½ï¿½sectorï¿½ï¿½
  * @param[in]      address: flash ï¿½ï¿½Ö·
  * @retval         sectorï¿½ï¿½
  */
uint32_t ger_sector(uint32_t address)
{
    uint32_t sector = 0;
    if ((address < ADDR_FLASH_SECTOR_1) && (address >= ADDR_FLASH_SECTOR_0))
    {
        sector = FLASH_SECTOR_0;
    }
    else if ((address < ADDR_FLASH_SECTOR_2) && (address >= ADDR_FLASH_SECTOR_1))
    {
        sector = FLASH_SECTOR_1;
    }
    else if ((address < ADDR_FLASH_SECTOR_3) && (address >= ADDR_FLASH_SECTOR_2))
    {
        sector = FLASH_SECTOR_2;
    }
    else if ((address < ADDR_FLASH_SECTOR_4) && (address >= ADDR_FLASH_SECTOR_3))
    {
        sector = FLASH_SECTOR_3;
    }
    else if ((address < ADDR_FLASH_SECTOR_5) && (address >= ADDR_FLASH_SECTOR_4))
    {
        sector = FLASH_SECTOR_4;
    }
    else if ((address < ADDR_FLASH_SECTOR_6) && (address >= ADDR_FLASH_SECTOR_5))
    {
        sector = FLASH_SECTOR_5;
    }
    else if ((address < ADDR_FLASH_SECTOR_7) && (address >= ADDR_FLASH_SECTOR_6))
    {
        sector = FLASH_SECTOR_6;
    }
    else if ((address < ADDR_FLASH_SECTOR_8) && (address >= ADDR_FLASH_SECTOR_7))
    {
        sector = FLASH_SECTOR_7;
    }
    else if ((address < ADDR_FLASH_SECTOR_9) && (address >= ADDR_FLASH_SECTOR_8))
    {
        sector = FLASH_SECTOR_8;
    }
    else if ((address < ADDR_FLASH_SECTOR_10) && (address >= ADDR_FLASH_SECTOR_9))
    {
        sector = FLASH_SECTOR_9;
    }
    else if ((address < ADDR_FLASH_SECTOR_11) && (address >= ADDR_FLASH_SECTOR_10))
    {
        sector = FLASH_SECTOR_10;
    }
    else if ((address < ADDR_FLASH_SECTOR_12) && (address >= ADDR_FLASH_SECTOR_11))
    {
        sector = FLASH_SECTOR_11;
    }
    else
    {
        sector = FLASH_SECTOR_11;
    }

    return sector;
}

/**
  * @brief          get the next page flash address
  * @param[in]      address: flash address
  * @retval         next page flash address
  */
/**
  * @brief          ï¿½ï¿½È¡ï¿½ï¿½Ò»Ò³flashï¿½ï¿½Ö·
  * @param[in]      address: flash ï¿½ï¿½Ö·
  * @retval         ï¿½ï¿½Ò»Ò³flashï¿½ï¿½Ö·
  */
uint32_t get_next_flash_address(uint32_t address)
{
    uint32_t sector = 0;

    if ((address < ADDR_FLASH_SECTOR_1) && (address >= ADDR_FLASH_SECTOR_0))
    {
        sector = ADDR_FLASH_SECTOR_1;
    }
    else if ((address < ADDR_FLASH_SECTOR_2) && (address >= ADDR_FLASH_SECTOR_1))
    {
        sector = ADDR_FLASH_SECTOR_2;
    }
    else if ((address < ADDR_FLASH_SECTOR_3) && (address >= ADDR_FLASH_SECTOR_2))
    {
        sector = ADDR_FLASH_SECTOR_3;
    }
    else if ((address < ADDR_FLASH_SECTOR_4) && (address >= ADDR_FLASH_SECTOR_3))
    {
        sector = ADDR_FLASH_SECTOR_4;
    }
    else if ((address < ADDR_FLASH_SECTOR_5) && (address >= ADDR_FLASH_SECTOR_4))
    {
        sector = ADDR_FLASH_SECTOR_5;
    }
    else if ((address < ADDR_FLASH_SECTOR_6) && (address >= ADDR_FLASH_SECTOR_5))
    {
        sector = ADDR_FLASH_SECTOR_6;
    }
    else if ((address < ADDR_FLASH_SECTOR_7) && (address >= ADDR_FLASH_SECTOR_6))
    {
        sector = ADDR_FLASH_SECTOR_7;
    }
    else if ((address < ADDR_FLASH_SECTOR_8) && (address >= ADDR_FLASH_SECTOR_7))
    {
        sector = ADDR_FLASH_SECTOR_8;
    }
    else if ((address < ADDR_FLASH_SECTOR_9) && (address >= ADDR_FLASH_SECTOR_8))
    {
        sector = ADDR_FLASH_SECTOR_9;
    }
    else if ((address < ADDR_FLASH_SECTOR_10) && (address >= ADDR_FLASH_SECTOR_9))
    {
        sector = ADDR_FLASH_SECTOR_10;
    }
    else if ((address < ADDR_FLASH_SECTOR_11) && (address >= ADDR_FLASH_SECTOR_10))
    {
        sector = ADDR_FLASH_SECTOR_11;
    }
    else /*(address < FLASH_END_ADDR) && (address >= ADDR_FLASH_SECTOR_23))*/
    {
        sector = FLASH_END_ADDR;
    }
    return sector;
}
