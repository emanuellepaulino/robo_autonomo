# Robô Autônomo

Neste projeto foi desenvolvido um robô capaz de se locomover de forma autônoma, por meio do processamento dos dados de interação sensorial com o ambiente, através da comunicação serial entre o ROS e o Arduino. O robô utilizou para detecção de obstáculos sensores ultrassônicos e a velocidade de uma de suas rodas pôde ser lida por meio de um sensor de velocidade. Os dados obtidos pelos sensores foram acessados de forma remota, enquanto o rob

### Material utilizado

- 1 x Arduino MEGA 2560
- 1 x Motor Shield L293D
- 4 x Motor JGB37-520 DC12V333RPM
- 3 x Sensor Ultrassônico HC-SR04
- 1 x Módulo Encoder Acoplador Óptico
- 1 x Disco Encoder
- 1 x Raspberry Pi 4
- 1 x Powerbank
- 1 x Fonte Externa de 12V
- 1 x Botão on/off
- 1 x Chassi
- 1 x Computador para acesso remoto ao robô

### Pré-requisitos

Para executar este projeto é necessário ter instalado algumas bibliotecas e frameworks. O projeto foi desenvolvido, testado e implementado no:

- Ubuntu 18.04

- ROS Melodic

- Arduino IDE

É necessário instalar no computador o Ubuntu e o ROS, e instalar no Raspberry Pi o Ubuntu, o ROS e a IDE do Arduino. É interessante ter a mesma versão do sistema operacional e do ROS em ambos, para que não tenham problemas com compatibilidade.

#### Dependências

Para realizar a comunicação serial entre o ROS e o Arduino utilizou-se o pacote *rosserial_arduino*. Sua instalação pode ser feita rodando os seguintes comandos no terminal do Ubuntu:

```
sudo apt-get install ros-${ROS_DISTRO}-rosserial-arduino
```

```
sudo apt-get install ros-${ROS_DISTRO}-rosserial
```

Foram utilizadas algumas bibliotecas para escrever o script. Essas bibliotecas fornecem funcionalidade extra e tornam o código mais simples. Bibliotecas utilizadas:

- *Ultrasonic.h* » biblioteca para os sensores ultrassônicos

- *AFMotor.h* » biblioteca para a motor shield

#### Habilitação do SSH

Tendo como finalidade acessar as informações dos sensores utilizados no robô enquanto o mesmo circula pelo ambiente, foi-se utilizado o Secure Shell, também chamado de SSH, um protocolo de rede que permite acessar e administrar de forma remota servidores.

Para realizar o acesso remoto foi necessário habilitar o SSH nas configurações do Raspberry e pegar o seu endereço IP por meio do comando. Os comandos necessários estão escritos abaixo:

Comando para acessar as configurações do Raspberry e habilitar o SSH:

```
sudo raspi-config
```

Comando para pegar o endereço IP do Raspberry:

```
 ifconfig
```

Comando a ser inserido no terminal do computador que irá ser utilizado durante o acesso remoto, no qual {user} é o nome do usuário root e {host} é o endereço IP do Raspberry:

```
ssh {user}@{host}
```

### Algoritmo desenvolvido para a navegação autônoma

Tal algoritmo foi desenvolvido a partir de funções declaradas que definem a movimentação do robô de acordo com os dados obtidos por meio dos sensores, seguindo a lógica presente no fluxograma abaixo:

   ![](/home/emanuelle/Downloads/Fluxograma.drawio.png) 
   
#### Montagem do robô




#### Algoritmo

    /*
    
     *  EMANUELLE GONÇALVES PAULINO
     *  ROSSERIAL COM SENSORES ULTRASSONICOS E SENSOR DE VELOCIDADE
        */
    
    #include <Ultrasonic.h>
    #include <AFMotor.h>
    #include <ros.h>
    #include <sensor_msgs/Range.h>
    #include <std_msgs/UInt16.h>
    
    ros::NodeHandle  nh;
    
    int d0 = 2;
    int rpm;
    volatile byte pulsos;
    unsigned long timeold;
    
    unsigned int pulsos_por_volta = 20;
    
    void contador() {
      pulsos++;
    }
    
    std_msgs::UInt16 rpm_msg;
    ros::Publisher chatter("rpm", &rpm_msg);
    
    // Declaração das posições de controle para os motores
    AF_DCMotor motor1(1);
    AF_DCMotor motor2(2);
    AF_DCMotor motor3(3);
    AF_DCMotor motor4(4);
    
    // Declaração dos sensores ultrassônicos
    Ultrasonic ultra_fre (40, 38);
    Ultrasonic ultra_esq (27, 26);
    Ultrasonic ultra_dir (46, 48);
    
    sensor_msgs::Range range_msg;
    
    sensor_msgs::Range range_front;
    sensor_msgs::Range range_left;
    sensor_msgs::Range range_right;
    
    ros::Publisher pub_range_front("/Sensor_da_frente", &range_front);
    ros::Publisher pub_range_left("/Sensor_da_esquerda", &range_left);
    ros::Publisher pub_range_right("/Sensor_da_direita", &range_right);
    
    void info_inic(sensor_msgs::Range &range_name, char *frameid)
    {
      range_name.radiation_type = sensor_msgs::Range::ULTRASOUND;
      range_name.header.frame_id = frameid;
      range_name.field_of_view = 0.26; // em radianos
      range_name.min_range = 2; // em cm
      range_name.max_range = 400; // em cm
    }
    
    void setup() {
    
      Serial.begin(9600);
    
      nh.initNode();
    
      pinMode(d0, INPUT);
      attachInterrupt(0, contador, FALLING);
      pulsos = 0;
      rpm = 0;
      timeold = 0;
    
      nh.advertise(chatter);
    
      motor1.setSpeed(110); 
      motor2.setSpeed(110);
      motor3.setSpeed(110); 
      motor4.setSpeed(110);
    
      nh.advertise(pub_range_front);
      nh.advertise(pub_range_left);
      nh.advertise(pub_range_right);
    
      info_inic(range_front, "/Sensor_da_frente");
      info_inic(range_left, "/Sensor_da_esquerda");
      info_inic(range_right, "/Sensor_da_direita");
    }
    
    void loop() {
    
      if (millis() - timeold >= 1000) {
    detachInterrupt(0);
    rpm = (60000 / pulsos_por_volta ) / (millis() - timeold) * pulsos;
    timeold = millis();
    pulsos = 0;
    
    rpm_msg.data = rpm;
    chatter.publish( &rpm_msg );
    
    attachInterrupt(0, contador, FALLING);
    }
        
      // Declaração de variáveis para as distâncias lidas pelos sensores ultrassônicos
      float dist_fre = ultra_fre.read();
      float dist_esq = ultra_esq.read();
      float dist_dir = ultra_dir.read();
    
      range_front.range = dist_fre;
      range_left.range = dist_esq;
      range_right.range = dist_dir;
    
      range_front.header.stamp = nh.now();
      range_left.header.stamp = nh.now();
      range_right.header.stamp = nh.now();
    
      pub_range_front.publish(&range_front);
      pub_range_left.publish(&range_left);
      pub_range_right.publish(&range_right);
    
      nh.spinOnce();
    
      // Desvio de obstáculos:
      //Não terá obstáculos em nenhuma direção
      if ((dist_fre > 40) and (dist_dir > 40) and (dist_esq > 40)) {
        frente();
      }
      // Obstáculo detectado à frente
      if ((dist_fre <= 40) and (dist_dir > 40)) {
        direita();
      }
      // Obstáculos detectados à frente e à direita
      if ((dist_fre <= 40) and (dist_dir <= 40) and (dist_esq > 40)) {
        esquerda();
      }
      //Obstáculos detectados em todas as opções
      if ((dist_fre <= 40) and (dist_dir <= 40) and (dist_esq <= 40)) {
        parar();
      }
    }
    
    // Funções declaradas para a movimentação do robô:
    
    void frente() {
      motor1.run(FORWARD); 
      motor2.run(FORWARD); 
      motor3.run(FORWARD); 
      motor4.run(FORWARD);
    }
    void parar() {
      motor1.run(RELEASE); 
      motor2.run(RELEASE); 
      motor3.run(RELEASE);
      motor4.run(RELEASE);
    }
    void direita() {
      motor1.run(FORWARD);
      motor2.run(BACKWARD); 
      motor3.run(BACKWARD);
      motor4.run(FORWARD);
    }
    
    void esquerda() {
      motor1.run(BACKWARD);
      motor2.run(FORWARD); 
      motor3.run(FORWARD);
      motor4.run(BACKWARD);
    }

