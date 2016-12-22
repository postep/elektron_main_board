#define BUFFER_LEN 256
#define BUFFER_DIV 0xff;


typedef struct {
	char buffer[BUFFER_LEN];
	int i, j;
}Buffer;

void buffer_init(Buffer *b){
	b->i = 0;
	b->j = 0;
}

void buffer_putn(char *c, int size, Buffer *b){
	int i = 0;
	while(size > 0){
		b->buffer[b->i] = c[i];
		++i;
		++(b->i);
		b->i &= BUFFER_DIV;
		--size;
	}
}

int buffer_put(char c, Buffer *b){
	int ret = (b->i == b->j);
	b->buffer[b->i] = c;
	++(b->i);
	b->i &= BUFFER_DIV;
	return ret;
}

int buffer_get(Buffer *b, char *c){
	int ret = (b->i != b->j);
	if (ret){
		*c = b->buffer[b->j];
		++(b->j);
		b->j &= BUFFER_DIV;
	}
	return ret;
}
