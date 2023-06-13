import board
import struct

i2c = board.STEMMA_I2C()

ADDRESS = 0x56

BUSY = 0x76
ACK = 0x79
NACK = 0x1F

class STMBootloader:
    def __init__(self, bus):
        self.bus = bus
        self._buf = memoryview(bytearray(258))

        self.supported_commands = self.get()[2:]
        print("read", " ".join(f"{x:02x}" for x in self.supported_commands))

        self.protocol_version = self.get_version()
        print("version", hex(self.protocol_version))

        self._chip_id = 0

    def get_version(self) -> int:
        # First fetch gets the length of the response
        self.write(b"\x01\xfe")
        self.wait_for_ack()
        v = self._read(1)[0]
        self.wait_for_ack()
        return v

    def get(self) -> bytes:
        # First fetch gets the length of the response
        self.write(b"\x00\xff")
        self.wait_for_ack()
        b = self._read(1)
        length = b[0]
        self.wait_for_ack()

        # Second fetch gets the full response
        self.write(b"\x00\xff")
        self.wait_for_ack()
        b = bytes(self._read(length + 2))
        self.wait_for_ack()
        return b

    @property
    def chip_id(self) -> int:
        if self._chip_id:
            return self._chip_id
        self.write(b"\x02\xfd")
        self.wait_for_ack()
        b = self._read(3)
        self._chip_id = struct.unpack_from(">H", b, offset=1)[0]
        self.wait_for_ack()
        return self._chip_id

    def wait_for_ack(self) -> bool:
        self._buf[0] = BUSY
        while self._buf[0] == BUSY:
            self.bus.readfrom_into(ADDRESS, self._buf, end=1)
        if self._buf[0] not in (NACK, ACK):
            raise RuntimeError()
        return self._buf[0] == ACK

    def write(self, buf, end=256):
        self.bus.writeto(ADDRESS, buf, end=end)

    def _checksum(self, data):
        if len(data) == 1:
            return (~data[0]) & 0xff
        cs = data[0]
        for v in data[1:]:
            cs ^= v
        return cs

    def _pack_address(self, address):
        struct.pack_into(">I", self._buf, 0, address)
        self._buf[4] = self._checksum(self._buf[:4])

    def read_memory(self, address, count):
        self.write(b"\x11\xee")
        self.wait_for_ack()
        self._pack_address(address)
        self.write(self._buf, end=5)
        if not self.wait_for_ack():
            return b""
        self._buf[0] = count - 1
        self._buf[1] = self._checksum(self._buf[:1])
        self.write(self._buf, end=2)
        self.wait_for_ack()
        return self._read(count)

    def go(self, address):
        """Go to the location of the interrupt table. (So branches to address
           + 4 because that's the location of the reset handler address.)"""
        self.write(b"\x21\xde")
        self.wait_for_ack()
        self._pack_address(address)
        self.write(self._buf, end=5)
        return self.wait_for_ack()

    def write_memory(self, address, data):
        """Go to the location of the interrupt table. (So branches to address
           + 4 because that's the location of the reset handler address.)"""
        if 0x32 in self.supported_commands:
            self.write(b"\x32\xcd")
        else:
            self.write(b"\x31\xce")
        self.wait_for_ack()
        self._pack_address(address)
        self.write(self._buf, end=5)
        if not self.wait_for_ack():
            return False
        self._buf[0] = len(data) - 1
        end = len(data) + 1
        self._buf[1:end] = data
        self._buf[end] = self._checksum(self._buf[:end])
        self.write(self._buf[:end+1])
        if not self.wait_for_ack():
            return False
        return True

    def _special_erase(self, command):
        if 0x45 in self.supported_commands:
            self.write(b"\x45\xBA")
        else:
            self.write(b"\x44\xBB")
        self.wait_for_ack()
        struct.pack_into(">H", self._buf, 0, command)
        self._buf[2] = self._checksum(self._buf[:2])
        self.write(self._buf[:3])
        return self.wait_for_ack()

    def erase_all(self):
        return self._special_erase(0xffff)

    def erase_bank(self, number):
        assert number in (1, 2)
        return self._special_erase(0xfffc + number)

    def erase_pages(self, pages):
        if len(pages) > 128:
            return False
        if 0x45 in self.supported_commands:
            self.write(b"\x45\xBA")
        else:
            self.write(b"\x44\xBB")
        self.wait_for_ack()
        struct.pack_into(">H", self._buf, 0, len(pages) - 1)
        self._buf[2] = self._checksum(self._buf[:2])
        self.write(self._buf[:3])
        if not self.wait_for_ack():
            return False
        for i, page in enumerate(pages):
            struct.pack_into(">H", self._buf, i * 2, page)
        l = 2 * len(pages)
        self._buf[l] = self._checksum(self._buf[:l])
        self.write(self._buf[:l])
        return self.wait_for_ack()

    def _read(self, count):
        self.bus.readfrom_into(ADDRESS, self._buf, end=count)
        return self._buf[:count]


i2c.try_lock()

stm = STMBootloader(i2c)
print(hex(stm.chip_id))
mv = stm.read_memory(0x08000000, 4)
# mv = stm.read_memory(0x40015800, 4)
print("before", hex(struct.unpack(">I", mv)))
stm.erase_pages((0,))
mv = stm.read_memory(0x08000000, 4)
# mv = stm.read_memory(0x40015800, 4)
print("after erase", hex(struct.unpack(">I", mv)))
# mv = stm.read_memory(0x0, 4)
print("write", stm.write_memory(0x08000000, b"\xad\xaf\x00\xad"))
mv = stm.read_memory(0x08000000, 4)
# mv = stm.read_memory(0x40015800, 4)
print("after", hex(struct.unpack(">I", mv)))
