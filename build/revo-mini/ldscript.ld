/* generated ldscript.ld */
MEMORY
{
    flash : org = 0x08010000, len = 960K
    ram0  : org = 0x20000000, len = 128k
}

INCLUDE common.ld
