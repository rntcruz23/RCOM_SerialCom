/*Non-Canonical Input Processing*/
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>


#define BAUDRATE B38400
#define MODEMDEVICE "/dev/ttyS1"
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1
#define clear() printf("\033[H\033[J");
#define FR_FLAG 0x7E
#define ESC 0x7D
#define ESCX 0x20

#define C_START 0x02
#define C_END 0x03
#define C_I 0x00

#define A_SENDER 0x03
#define A_RECEIVER 0x01

#define C_SET 0x03
#define C_DISC 0x0B

#define C_UA 0x07
#define C_RR 0x05
#define C_REJ 0x01
#define I0 0x00
#define I1 0x40
#define MAX 1024
#define MAXPACK 256

volatile int STOP = FALSE;
int flag_ack = 0, flag_alarm = 0;

int llopen(int porta, int status);
int trama_construct(char trama[], char origin, char control);
int wait_ack(int porta);
char process_trama(char *trama, int size, int porta);
int test_bcc1(char *trama);           // Testa flag BCC1
int test_bcc2(char *trama, int size); // Testa flag BCC2
char proc_C(char *trama, int porta);  // Processa flag C em trama
int user_input(char *packet);         // Le input do user
int info_construct(char *buf, char *packet, int packet_size,
                   int n);                 // Insert packet(buf) in trama
int leitura_trama(char *trama, int porta); // Le porta serie ate encontrar trama
void create_UA(char *trama);
void rr_construct(char *trama, int rr);
void disc_construct(char *trama);
void rej_construct(char *trama, int sequence_number);
int wait_disc(int porta);
int sendFile(char *filename, int porta);
int getFileLength(char type, int length, char *camada_app);
int getFileName(char type, int length, char *camada_app, char *filename);
int createApplevelStart(char *filename, int size, char *appLevel);
int wait_sequence(int porta, int Ns, char *appLevel, int applevelSize);
int stuffFrame(char *trama, int size, char *out);
int destuff(char *packet, int packet_size);

int llwrite(int fd, char *buffer, int length);
int llread(int fd, char *buffer);
int llclose(int fd);

typedef struct {
  int fd;
  int status;
} applicationLayer;
typedef struct {
  char port[20];               /*Dispositivo /dev/ttySx, x = 0, 1*/
  int baudRate;                /*Velocidade de transmissão*/
  unsigned int sequenceNumber; /*Número de sequência da trama: 0, 1 : trama que
                                  recetor espera receber*/
  unsigned int timeout;        /*Valor do temporizador: 3 s*/
  unsigned int numTransmissions; /*Número de tentativas em caso de falha*/
  char frame[MAX];               /*Trama*/
} linkLayer;
linkLayer linkL;
applicationLayer app;
void resend() {
  if (!flag_ack) { // Resend
    flag_alarm = 0;
    return;
  }
  flag_ack = 1; // ack ja recebido
  return;
}
int main(int argc, char **argv) {
  linkL.baudRate = BAUDRATE;
  linkL.sequenceNumber = 0;
  linkL.timeout = 3;
  linkL.numTransmissions = 3;
  int res;
  struct termios oldtio, newtio;
  char buf[MAX];
  char *filename;
  // unsigned char trama[1024];
  if (argc == 3) {
    app.status = A_SENDER;
    filename = argv[2];
  } else if (argc == 2)
    app.status = A_RECEIVER;
  else if ((argc < 2) || ((strcmp("/dev/ttyS0", argv[1]) != 0) &&
                          (strcmp("/dev/ttyS1", argv[1]) != 0))) {
    printf("Usage:\tnserial SerialPort\n\tex: nserial /dev/ttyS1\n");
    exit(1);
  } else {
    printf("Wrong input\n");
    return -1;
  }

  /*
    Open serial port device for reading and writing and not as controlling tty
    because we don't want to get killed if linenoise sends CTRL-C.
  */
  app.fd = open(argv[1], O_RDWR | O_NOCTTY);
  if (app.fd < 0) {
    perror(argv[1]);
    exit(-1);
  }

  if (tcgetattr(app.fd, &oldtio) == -1) { /* save current port settings */
    perror("tcgetattr");
    exit(-1);
  }
  bzero(&newtio, sizeof(newtio));
  newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;
  /* set input mode (non-canonical, no echo,...) */
  newtio.c_lflag = 0;
  newtio.c_cc[VTIME] = 1; /* inter-character timer unused */
  newtio.c_cc[VMIN] = 0;  /* blocking read until 5 chars received */
                          /*
                            VTIME e VMIN devem ser alterados de forma a proteger com um temporizador a
                            leitura do(s) próximo(s) caracter(es)
                          */
  tcflush(app.fd, TCIOFLUSH);
  if (tcsetattr(app.fd, TCSANOW, &newtio) == -1) {
    perror("tcsetattr");
    exit(-1);
  }
  printf("New termios structure set\n");
  // output_file = open("output.txt", O_RDWR | O_CREAT);
  if (llopen(app.fd, app.status) >=
      0) { // emissor -> send SET    recetor -> wait SET send ACK 	return: -1
           // on failure
    switch (app.status) { // SET enviado ACK recebido
    case A_RECEIVER:
      res = llread(app.fd, buf); // llread()
      break;
    case A_SENDER:
      if ((res = sendFile(filename, app.fd)) == -1)
        printf("Error sending file: connection timeout\n");
      break;
    }
  } else
    printf("Problema na inicializacao (llopen())\n");
  llclose(app.fd);
  printf("%d bytes written\n", res);
  /*
    O ciclo FOR e as instruções seguintes devem ser alterados de modo a
    respeitar o indicado no guião
  */
  if (tcsetattr(app.fd, TCSANOW, &oldtio) == -1) {
    perror("tcsetattr");
    exit(-1);
  }
  close(app.fd);
  return 0;
}
int llopen(int porta, int status) {
  char trama[5];
  int readsize, flag = 0;
  if (status == A_SENDER) {
    printf("Sending SET...\nWaiting UA...\n");
    if (wait_ack(porta))
      return porta;
  } else if (status == A_RECEIVER) {
    printf("Waiting communication...\n");
    while (flag != C_SET) {
      readsize = leitura_trama(trama, porta);       // RECEBE A TRAMA
      flag = process_trama(trama, readsize, porta); // processa trama recebida
      if (flag == C_DISC)
        return -1;
    }
    printf("Sending UA...\n");
    return porta;
  }
  return -1;
}
int wait_ack(int porta) { // return: 0 -> max trys | 1 -> ack
  char size;
  char trama[MAX];
  int send_count = 0;
  flag_ack = flag_alarm = 0;
  (void)signal(SIGALRM, resend); // Rotina para timeout
  while (!flag_ack) {            /* loop for input */
    if (!flag_alarm) {
      int i = trama_construct(trama, A_SENDER, C_SET);
      write(porta, trama, i);
      send_count++;
      if (send_count > linkL.numTransmissions)
        break;
      if (send_count > 1)
        printf("Try %d\n", send_count);
      flag_alarm = 1;
      alarm(linkL.timeout);
    }
    size = leitura_trama(trama, porta);
    if (!flag_alarm)
      continue;
    flag_ack = (process_trama(trama, size, porta) == C_UA); // Test for ACK
    if (!flag_ack) { // Nova trama invalida
      flag_alarm = 0;
      send_count = 0;
      alarm(0);
    } else if (flag_ack) // Nova trama esperada
      break;
    if (flag_ack == -1)
      printf("Trama invalida!\n");
  }
  return ((send_count <= linkL.numTransmissions) && flag_ack);
}
char process_trama(char *trama, int size, int porta) { // Return flag C
  int test;
  char c_flag;
  if ((test = test_bcc1(trama)) == -1)
    return -1;
  c_flag = proc_C(trama, porta);
  return c_flag; // Trama de controlo
}
int test_bcc1(char *trama) { // Return 1 -> trama de controlo || Return 0 ->
                             // trama de informaçao
  if ((trama[1] ^ trama[2]) != trama[3])
    return -1;
  return trama[2] & 1;
}
int test_bcc2(char *camada_app, int size) { // FAC[dados|bcc2]F
  char bcc2 = camada_app[size - 1];
  char xor_bcc2 = camada_app[0];
  int i = 1;
  while (i != size - 1)
    xor_bcc2 ^= camada_app[i++];
  if (xor_bcc2 != bcc2)
    return -1;
  return 1;
}
char proc_C(char *trama, int porta) { // Return flag C
  char c = trama[2];
  char response_trama[5];
  switch (c) {
  case C_SET:
    create_UA(response_trama);
    write(porta, response_trama, 5);
    printf("SET Received\n");
    break;     // SEND UA
  case C_DISC: // Close connection
    create_UA(response_trama);
    write(porta, response_trama, 5);
    printf("Received Disc\n");
    break;
  case C_UA:
    flag_ack = 1;
    printf("UA Received\n"); // Wait for data
    break;
  case C_REJ:
    break;
  default:
    printf("Frame Unknown!");
  }
  return c;
}
int info_construct(char *buf, char *packet, int packet_size,
                   int n) { // Insert packet(buf) in trama

  char L1, L2;
  int i;

  if (buf == NULL || packet == NULL)
    return -1;

  packet[0] = 0;
  packet[1] = n;

  L1 = (char)packet_size;
  L2 = packet_size >> 8;

  packet[2] = L2;
  packet[3] = L1;

  for (i = 0; i < packet_size; i++) {
    packet[i + 4] = buf[i];
  }

  return 0;
}
int leitura_trama(char *trama, int porta) {
  int start = 0, fim = 0, cnt = 0;
  char ch = 0;
  while (!(fim) && (cnt < MAX) && !(!flag_alarm * (app.status == A_SENDER))) {
    read(porta, &ch, 1);
    if ((cnt == 1) && (ch == FR_FLAG))
      cnt = 0;                              // FR_FLAG seguido de FR_FLAG
    else if (ch == FR_FLAG && start == 0) { // Inicio de uma trama
      start = 1;
    } else if (ch == FR_FLAG && start == 1) { // Fim de uma trama
      start = 0;
      fim = 1;
    }
    if (start == 1)
      trama[cnt++] = ch;
  }
  trama[cnt++] = FR_FLAG;
  return (cnt >= MAX ? -1 : cnt);
}
void create_UA(char *trama) {
  trama[0] = trama[4] = FR_FLAG;
  trama[1] = app.status;
  trama[2] = C_UA;
  trama[3] = trama[1] ^ trama[2];
}
int sendFile(char *filename, int porta) {
  int filesize = 0, n = 0, i = 0, npack = 0, j = 0;
  char appLevel[MAXPACK];
  char beforeStuff[MAXPACK + 4];
  char chunck[MAXPACK];
  FILE *f;
  size_t nread;
  char packet[2 * (MAXPACK + 4)];
  f = fopen(filename, "r");
  if (f == NULL) {
    printf("ERROR OPENING FILE\n"); /* deal with error */
  }

  fseek(f, 0, SEEK_END);
  filesize = ftell(f);
  printf("Filesize: %d\n", filesize);
  fseek(f, 0, SEEK_SET);

  int applevelSize = createApplevelStart(
      filename, filesize,
      beforeStuff); // Cria camada de aplicaçao F|A|C|B[DADOS]B|F
  applevelSize =
      stuffFrame(beforeStuff, applevelSize,
                 appLevel); // Stuff frame and create BCC2 F|A|C|B[DADOS|B]F

  if (!wait_sequence(porta, linkL.sequenceNumber, appLevel, applevelSize))
    return -1; // wait rr1

  if (f) {
    while ((nread = fread(chunck, 1, sizeof chunck, f)) > 0) {
      info_construct(chunck, beforeStuff, nread, n);
      applevelSize = stuffFrame(beforeStuff, nread + 4, packet);
      if (!wait_sequence(porta, linkL.sequenceNumber, packet, applevelSize))
        return -1; // wait rr1
      i = nread;
      n++;
      npack += nread;
      clear();
      printf("\tPacket: #%d\tSent: %0.0f%%\n[", n,
             ((float)npack / (float)filesize) * 100);
      while (j++ < 50) {
        if (j <= ((float)npack / (float)filesize) * 50)
          printf("#");
        else
          printf(" ");
      }
      printf("]\n");
      j = 0;
    }
  }

  fclose(f);

  // Ready to send file
  // Dividir ficheiro em pacotes [P1...Pk]
  // CRIAR TRAMAS DE INFORMAÇAO -> camada de aplicaçao [DADOS] ->
  // [C|N|L2|L1|P1...Pk] llwrite -> acrescentar cabeçalhos e enviar ->
  // [F|A|C|B]DADOS[B|F] -> n++;
  return (n - 1) * MAXPACK + i;
}
int stuffFrame(char *trama, int size, char *out) {
  int i, j = 0;
  char xor = trama[0];
  for (i = 0; i < size; i++) {
    if (i)
      xor ^= trama[i];
    if (trama[i] == FR_FLAG) {
      out[j++] = ESC;
      out[j++] = FR_FLAG ^ ESCX;
    } else if (trama[i] == ESC) {
      out[j++] = ESC;
      out[j++] = ESC ^ ESCX;
    } else
      out[j++] = trama[i];
  }
  if (xor == FR_FLAG) {
    out[j++] = ESC;
    out[j++] = FR_FLAG ^ ESCX;
  } else if (xor == ESC) {
    out[j++] = ESC;
    out[j++] = ESC ^ ESCX;
  } else
    out[j++] = xor;
  return j;
}
int wait_sequence(int porta, int Ns, char *appLevel, int applevelSize) {
  int send_count = 0;
  char trama[MAX];
  flag_ack = flag_alarm = 0;
  while (!flag_ack) { /* loop for input */
    if (!flag_alarm) {
      llwrite(porta, appLevel,
              applevelSize); // Adiciona cabeçalhos e envia [F|A|C|B]DADOS[B|F]
      flag_alarm = 1;
      send_count++;
      if (send_count > linkL.numTransmissions)
        break;
      if (send_count > 1)
        printf("Try %d\n", send_count);
      alarm(linkL.timeout);
    }
    leitura_trama(trama, porta);
    if (!flag_alarm)
      continue;
    alarm(0);
    char Ns = ((linkL.sequenceNumber ^ 1) << 6);
    char NsBefore = linkL.sequenceNumber << 6;
    char expected = Ns ^ C_RR;
    flag_ack = (trama[2] == expected); // Test for RR[Ns]
    if (trama[2] == (NsBefore ^ C_REJ)) {
      send_count = 0;
      printf("Rej received... Resending\n");
      llwrite(porta, appLevel, applevelSize);
      flag_alarm = 1;
      alarm(3);
    }
    if (flag_ack)
      alarm(0);
    if (flag_ack == -1)
      printf("Trama invalida!\n");
  }
  linkL.sequenceNumber ^= 1;
  return ((send_count < linkL.numTransmissions) && flag_ack);
}
int llwrite(int fd, char *buffer, int length) {
  char trama[5];
  int i, written = 0;
  trama[0] = trama[4] = FR_FLAG;
  trama[1] = app.status;
  trama[2] = linkL.sequenceNumber << 6;
  trama[3] = trama[1] ^ trama[2];
  for (i = 0; i < 4; i++)
    written += write(fd, trama + i, 1);
  for (i = 0; i < length; i++)
    written += write(fd, buffer + i, 1);
  written += write(fd, trama + 4, 1);
  return written;
}
int createApplevelStart(char *filename, int size, char *appLevel) {
  int namesize = strlen(filename);
  int i;
  appLevel[0] = C_START;
  appLevel[1] = 0x00;
  appLevel[2] = (char)sizeof(int);
  for (i = sizeof(int) + 2; i > 2; i--) {
    appLevel[i] = (char)size;
    size = size >> 8;
  }
  appLevel[7] = 0x01;
  appLevel[8] = (char)namesize;
  for (i = 0; i < namesize; i++) {
    appLevel[i + 9] = filename[i];
  }
  return namesize + 5 + appLevel[2];
}
int llread(int fd, char *buffer) {
  int readsize, packet_size;
  char rr_trama[5];
  char *packet;
  char c_info = 0, c_control = 0, type;
  int lengthSize, lengthName, filesize, output_file, nFrame, received = 0;
  int frameSize, j = 0;
  char filename[MAX];
  while (c_control != C_DISC || c_info == C_END) {
    if ((readsize = leitura_trama(buffer, fd)) != -1) {
      packet_size = readsize - 5;
      packet = buffer + 4;
      if (test_bcc1(buffer) != -1) { // BCC1 correto
        c_control = buffer[2];
        if ((c_control & 1) == 1)
          continue;
        if ((int)c_control >> 6 ==
            linkL.sequenceNumber) { // Trama recebida tem numero de sequencia
                                    // esperado
          packet_size = destuff(packet, packet_size);
          if (test_bcc2(packet, packet_size) > 0) { // BCC2 correto
            c_info = packet[0];
            switch (c_info) {
            case C_START:
              printf("START frame\n");
              type = packet[1];
              lengthSize = (int)packet[2];
              if ((filesize = getFileLength(type, lengthSize, packet + 3)) ==
                  -1)
                return -1;
              type = packet[3 + lengthSize];
              lengthName = packet[3 + lengthSize + 1];
              if (getFileName(type, lengthName, packet + 3 + lengthSize + 2,
                              filename) == -1)
                return -1;
              printf("Filesize: %d:%d:%d\tFilename: %s\n", filesize, lengthName,
                     lengthSize, filename);
              output_file =
                  open(filename, O_RDWR | O_CREAT); // ficheiro de saida criado
              break;
            case C_I:
              nFrame = packet[1];
              frameSize = packet[2] * 256 + packet[3];
              if (frameSize < 0)
                frameSize = frameSize + 256;
              received += frameSize;
              clear();
              printf("\tFrame: #%d Received: %0.0f%%\n[", nFrame,
                     ((float)received / (float)filesize) * 100);
              while (j++ < 50) {
                if (j <= ((float)received / (float)filesize) * 50)
                  printf("#");
                else
                  printf(" ");
              }
              printf("]\n");
              j = 0;
              write(output_file, packet + 4, frameSize);

              break;
            case C_END:
              printf("END frame\n");
              break;
            default:
              return -1;
            }
            linkL.sequenceNumber ^= 1; // Mudar numero de trama esperado
            rr_construct(rr_trama, linkL.sequenceNumber); // Pedir trama
                                                          // seguinte
          } else
            rej_construct(rr_trama, linkL.sequenceNumber); // BCC2 errado
        } else
          rr_construct(rr_trama,
                       linkL.sequenceNumber); // Numero de sequencia nao
                                              // esperado, pedir novamente
      } else
        rej_construct(rr_trama, linkL.sequenceNumber); // BCC1 errado
      write(fd, rr_trama, 5); // Enviar trama de resposta
    }
  }
  close(output_file);
  return received;
}

int getFileName(char type, int length, char *camada_app, char *filename) {
  int i;
  if (!camada_app || !filename)
    return -1;
  switch (type) {
  case 0x01:
    for (i = 0; i < length; i++)
      filename[i] = camada_app[i];
    break;
  default:
    return -1;
  }
  filename[i] = 0;
  return length;
}
int getFileLength(char type, int length, char *camada_app) {
  int filesize = 0;
  int i;
  unsigned char aux;
  switch (type) {
  case 0x00:
    for (i = 0; i < length; i++) {
      aux = camada_app[i];
      filesize += (int)aux << ((length - i - 1) * 8);
    }
    break;
  default:
    return -1;
  }
  return filesize;
}
int llclose(int fd) {
  char rr_trama[5];
  disc_construct(rr_trama); // wait DISC  ->send ack();
  if (app.status == A_SENDER) {
    if (!wait_disc(fd))
      return -1;
  } else if (app.status ==
             A_RECEIVER) { // DISC recebido -> send DISC -> wait_ack();
    write(fd, rr_trama, 5);
    if (!wait_ack(fd))
      return -1;
  }
  return 1;
}
void rr_construct(char *trama, int sequence_number) {
  trama[0] = trama[4] = FR_FLAG;
  trama[1] = app.status;
  trama[2] = (sequence_number << 6) ^ C_RR;
  trama[3] = trama[1] ^ trama[2];
}
int trama_construct(char trama[], char origin, char control) {
  // TRAMA = [F][A][C][CS][PACKET][CS][FF]
  //          0  1  2  3
  trama[0] = trama[4] = FR_FLAG;
  trama[1] = A_SENDER;
  trama[2] = C_SET;
  trama[3] = trama[1] ^ trama[2];
  return 5;
}
void rej_construct(char *trama, int sequence_number) {
  trama[0] = trama[4] = FR_FLAG;
  trama[1] = app.status;
  trama[2] = (sequence_number << 6) ^ C_REJ;
  trama[3] = trama[1] ^ trama[2];
}
void disc_construct(char *trama) {
  trama[0] = trama[4] = FR_FLAG;
  trama[1] = app.status;
  trama[2] = C_DISC;
  trama[3] = trama[1] ^ trama[2];
}
int destuff(char *packet, int packet_size) {
  int i, cnt = 0, curr = 0;
  char *new_packet = packet;
  for (i = 0; i < packet_size; i++) {
    if (packet[i] == ESC) {
      curr--;
      cnt++;
    } else if ((packet[i] == (ESC ^ ESCX)) && cnt) {
      new_packet[curr] = ESC;
      cnt = 0;
    } else if ((packet[i] == (FR_FLAG ^ ESCX)) && cnt) {
      new_packet[curr] = FR_FLAG;
      cnt = 0;
    } else
      new_packet[curr] = packet[i];
    curr++;
  }
  return curr;
}
int wait_disc(int porta) { // return: 0 -> max trys | 1 -> ack
  char size;
  char rr_trama[5];
  int send_count = 0;
  char trama[MAX];
  disc_construct(rr_trama);
  printf("Waiting DISC\n");
  flag_ack = flag_alarm = 0;
  (void)signal(SIGALRM, resend); // Rotina para timeout
  while (!flag_ack) {            /* loop for input */
    if (!flag_alarm) {
      flag_alarm = 1;
      write(porta, rr_trama, 5);
      send_count++;
      if (send_count > linkL.numTransmissions)
        break;
      if (send_count > 1)
        printf("Try %d\n", send_count);
      alarm(linkL.timeout);
    }
    size = leitura_trama(trama, porta);
    if (!flag_alarm)
      continue;
    alarm(0);
    flag_ack = (process_trama(trama, size, porta) == C_DISC); // Test for DISC
    if (!flag_ack) {
      send_count = 0;
      flag_alarm = 0;
    } else
      break;
    if (flag_ack == -1)
      printf("Trama invalida!\n");
  }
  printf("Disconect received\n");
  return ((send_count <= linkL.numTransmissions) && flag_ack);
}
