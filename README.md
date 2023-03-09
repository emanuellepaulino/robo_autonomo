# Robô Autônomo

Neste projeto foi desenvolvido um robô capaz de se locomover de forma autônoma, por meio do processamento dos dados de interação sensorial com o ambiente, através da comunicação serial entre o ROS e o Arduino. 

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

`sudo apt-get install ros-${ROS_DISTRO}-rosserial-arduino`

`sudo apt-get install ros-${ROS_DISTRO}-rosserial`

Foram utilizadas algumas bibliotecas para escrever o script. Essas bibliotecas fornecem funcionalidade extra e tornam o código mais simples. Bibliotecas utilizadas:

- *Ultrasonic.h* » biblioteca para os sensores ultrassônicos

- *AFMotor.h* » biblioteca para a motor shield

