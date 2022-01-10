import struct
import sys

def as_bytes(num: float):
	packed_float = struct.pack("<f", num)
	# print(packed_float.hex())
	print(f"{num}: bytes to pass (in order):")
	for b in packed_float:
		print(f"{b:08b}")
	a = packed_float[0] | (packed_float[1] << 8) | (packed_float[2] << 16) | (packed_float[3] << 24)
	# print(hex(a))
	packed_uint = struct.pack("=L", a)
	# print(packed_uint.hex())
	unpacked_float = struct.unpack("=f", packed_uint)
	# print(unpacked_float)
	# check if the packed data correctly represents the number converted to a (32-bit) float
	assert unpacked_float[0] == struct.unpack("f", struct.pack("f", num))[0]

if __name__ == "__main__":
	if len(sys.argv) > 1:
		for arg in sys.argv[1:]:
			as_bytes(float(arg))
	else:
		num = float(input("type a float:"))
		as_bytes(num)
