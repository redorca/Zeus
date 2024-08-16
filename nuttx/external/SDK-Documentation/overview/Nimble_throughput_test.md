@page Performance Performance
@cond (BLE_Perf)

 ### The theoretical throughput analysis:
    * 1M/s = 1Mbps = 1000Kbps = 125000Bps = 125B/ms
    * 2M/s = 2Mbps = 2000Kbps = 250000Bps = 250B/ms
    * This is the theropy value, include all layer head and check bits.
    * Pure Payload = MTU- (2(ATT handler ) - 1(opcode)) = 20

  TX  | RX       | Software | Connection interval | Data TX interval | MTU | Distance | Data length extension | PHY  | Expentation(Pure payload)  | Result(Pure payload) | Spare bandwith
 ----|----------|---------------------|------------------|-----|----------|-----------------------|------|----------------------------|----------------------|---------|----------
  Nordic52840 | Nordic52840 | ZDK | 10ms | 10ms | 32B | 1M | NA | 1M | 2.9KBps | 2.6KBps | @image html TestRun_1_image002.png 
  Nordic52840 | Nordic52840 | ZDK | 10ms | 10ms | 32B | 10M | NA | 1M | 2.9KBps | 2.5KBps | @image html TestRun_2_image004.png
  Nordic52840 | Nordic52840 | ZDK | 10ms | 3ms | 32B | 1M | NA | 1M | 8.7kBps | 5.5 KBps | @image html TestRun_3_image006.png
  Nordic52840 | Nordic52840 | ZDK | 10ms | 3ms | 32B | 10M | NA | 1M | 8.7kBps | 3.8KBps | @image html TestRun_4_image009.png
  Nordic52840 | Nordic52840 | ZDK | 10ms | 10ms | 200B | 1M | NA | 1M | 19.7KBps | 17.5KBps | @image html TestRun_5_image010.png
  Nordic52838 | Nordic52840 | ZDK | 10ms | 10ms | 300B | 10M | NA | 1M | 19.7KBps | 11KB/s | @image html TestRun_6_image012.png

@endcond
