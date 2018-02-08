Bootloader testcases
1. Normal/cold power up
  1.1 BT module is started
  1.2 test BT module

2. CPU Short Power discconnect Reset
  2.1 BT is connected
  2.1.1 should continue normally

3. Firmware update / CPU medium power down time / forced update
  3.1 reset BT module
  3.2 test UART speed
  3.3 update BT module settings
  3.4 wait up to 20 seconds until new connection, either updater or console connection

4. Firmware update with nonfunctional firmware


5. BT module failure detection  
