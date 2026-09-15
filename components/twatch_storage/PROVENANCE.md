# NVS Storage provenance

Copied key_value_nvs_impl.{cpp,hpp} and private/nvs_flash_manager.{cpp,hpp}
from ESP-Brookesia hal/brookesia_hal_adaptor/src at commit
2c09459a97a82c8d477c543eb2b43f4956b8a5be, Apache-2.0.
Original copyright headers retained. Local changes enable only the KV backend,
provide a local log tag, and remove automatic whole-partition erasure from NVS
initialization. Errors are returned to the caller without erasing stored data.
The board bridge owns registration; Storage service owns initialization and use.
No filesystem or extra peripheral is enabled. This isolated copy avoids enabling
the full HAL adaptor and should be replaced by upstream when it offers a
non-erasing initialization policy. Explicit namespace/key erase methods remain
part of the normal KV API; they are not invoked as recovery behavior.
