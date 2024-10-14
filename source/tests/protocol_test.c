/* Simple Protocol test program */

#include <stdio.h>
#include <string.h>
#include "../protocol.h"

void test (protocol_t* p);

int main (void)
{
	protocol_t p;

	p = (protocol_t){ 'A', 123 };
	test(&p);
	p = (protocol_t){ 'B', +12 };
	test(&p);
	p = (protocol_t){ 'b', -1 };
	test(&p);
	p = (protocol_t){ '1', -9999 };
	test(&p);
	p = (protocol_t){ '1', -321 };
	test(&p);
	p = (protocol_t){ 49, -0 };
	test(&p);

	return 0;
}

void test (protocol_t* p)
{
	uchar_t msg[PROTOCOL_DIGS];
	uint8_t len;

	printf("Original angles: %c, %d\n", p->id, p->val);
	len = protocolPack(p, msg);
	printf("Packed message: ");
	for(int i = 0; i < len; i++)
		printf("%c", (msg)[i]);
	printf("\n");
	p = protocolUnpack(msg, len);
	printf("Unpacked angles: %c, %d\n", p->id, p->val);
}
