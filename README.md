# ponderada_turtlebot_teleop

Este é um programa Python para controlar um TurtleBot3 usando o ROS 2 (Robot Operating System). O programa permite controlar o robô usando o teclado, ajustando a velocidade linear e angular em tempo real.

## Movimentação do Robô

O controle será feito pelas teclas de seta:

Seta para cima: Robo vai para frente
Seta para baixo: Robo vai para trás
Seta para direita: Robo gira em torno do seu eixo para a direita
Seta para esquerda: Robo gira em torno do seu eixo para a esquerda

"Espaço": Mata o nó de comunicação com o ROS
Tecla "S": Parada de emergência do Robô

## Pré-requisitos

Certifique-se de ter o ROS 2 instalado no seu sistema. Além disso, é necessário preparar a área de trabalho para trabalhar com o ROS 2.

1. Instale o ROS seguindo as instruções no site oficial: [ROS Installation](http://wiki.ros.org/Installation).

2. Rode esses comando para poder preparar a área do seu pc para funcionar:
(Esse comando serve para poder rodar o WEBOTS que é um ambiente virtual para testar os códigos no Robô)
```
ros2 launch webots_ros2_turtlebot robot_launch.py
```

3. Prepare a área de trabalho executando os seguintes comandos no terminal:

```bash
colcon build
source install/local_setup.bash
```

4. Para poder baixar todas as dependências usamos o código:
```bash
python3 -m pip install -r requirements.txt
```

## Intalação do WeBots (ambiente virtual para testes do robô)

Para usar o WeBots, é necessário preparar o ambiente executando os seguintes comandos:

1. Abra o terminal e rode os comandos a seguir:

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3*

sudo apt install ros-humble-rmw-cyclonedds-cpp

echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
```
2. Depois de rodar esses comandos:

```bash
sudo mkdir -p /etc/apt/keyrings
cd /etc/apt/keyrings
sudo wget -q https://cyberbotics.com/Cyberbotics.asc
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/" | sudo tee /etc/apt/sources.list.d/Cyberbotics.list
```

```bash
sudo apt update
sudo apt install webots
sudo apt install ros-humble-webots-ros2
```

## Executando o Programa

Após preparar o ambiente, você pode executar o programa para controlar a tartaruga no Turtlesim. Execute o seguinte comando no terminal:
(É bem importante estar no local da pasta certa que é (/home/murilo_prianti/Documents/GitHub/ponderada_turtlebot_teleop/Ponderada_sem_2/src/backend))

```bash
python3 movimentacao.py
```

## Detalhes do Projeto

O objetivo do projeto é criar uma aplicação que permitam as pessoas que vão usar o código, enviar comandos de movimento para a tartaruga no Turtlesim ( em um formato de vx(0.0), vy(0.0) vtheta(0.0) e time(1000)). Esses comandos estão sendo enfileirados e são executados sequencialmente, garantindo que cada comando seja concluído antes que o próximo seja iniciado com o uso do deque que fica presente no collections que ja vem com o python. Dessa forma, podemos proporcionar uma experiência de uso fluida para os operadores.

## Autor

Este projeto foi desenvolvido por mim, Murilo de Souza Prianti Silva.