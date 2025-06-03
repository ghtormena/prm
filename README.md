# PRM - Programação de Robôs Móveis

**Disciplina SSC0712**  

Este repositório contém o Projeto 1 da disciplina *Programação de Robôs Móveis*, focado no desenvolvimento de soluções de navegação autônoma em ambientes simulados utilizando **ROS 2 Humble** e o simulador **Gazebo Fortress**.  

O robô é capaz de identificar uma bandeira, alinhar-se a ela e evitar obstáculos no caminho utilizando LiDAR e visão computacional.

---

## 📦 Tecnologias utilizadas

- ROS 2 Humble
- Gazebo Fortress
- Python
- OpenCV
- RViz
- CVBridge

---

## 🚀 Como utilizar o pacote

### 1. Clonar o repositório

Acesse a pasta `src` do seu workspace ROS 2:

```bash
cd ~/ros2_ws/src/
git clone https://github.com/ghtormena/prm.git
````

### 2. Instalar dependências

Instale as dependências do pacote com:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

> Certifique-se de ter rodado previamente `sudo rosdep init` e `rosdep update`, se for a primeira vez usando o `rosdep`.

### 3. Compilar o workspace

Certifique-se de estar na **raiz do seu workspace** (geralmente `~/ros2_ws`) antes de compilar:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select prm
```

### 4. Atualizar o ambiente do terminal

```bash
source install/local_setup.bash
```

---

## 🧪 Executando a simulação

### 1. Iniciar o mundo no Gazebo

```bash
ros2 launch prm inicia_simulacao.launch.py
```

### 2. Carregar o robô no ambiente

Em um **novo terminal** (não se esqueça de `source install/local_setup.bash`):

```bash
ros2 launch prm carrega_robo.launch.py
```

### 3. Controle 

Em outro terminal:

```bash
ros2 run prm controle_robo_novo
```

### 4. Lógica de Controle e Estados do Robô
A lógica de controle do robô é baseada em LiDAR e visão computacional. Os principais estados de operação são:

- Movendo reto: o robô anda para frente quando não detecta obstáculos nem bandeira.
- Desviando de obstáculo sem bandeira: desvia de obstáculos sem detectar a imagem da bandeira.
- Girando parado (obstáculo muito próximo): se algo estiver a menos de 20 cm, gira no próprio eixo.
- Movendo em direção à bandeira: move-se em direção à bandeira ajustando a centralização.
- Desviando com bandeira visível: se a bandeira está visível mas há obstáculo, o robô realiza desvio de forma mais suave para não perder a bandeira de vista.
- Girando para centralizar a bandeira próxima: quando está chegando perto da bandeira, gira parado até alinhar-se horizontalmente com ela.
- Missão cumprida: ao centralizar e se aproximar da bandeira, o robô para completamente.

### 5. Autores
- Giovanna Herculano Tormena - 12674335
- Vinicius Gustierrez Neves - 14749363
- Guilherme Rebecchi - 12550107
