# bayesian_agent

Реализация агентов команды спасателей для **RoboCup Rescue Agent Simulation (RCRS)**,
построенная поверх [adf-core-java](https://github.com/roborescue/adf-core-java).

Тема ВКР: «Компонент принятия решений о маршруте спасения в критических ситуациях с использованием байесовских методов».

Используется полный стек байесовского планирования:
байесовское убеждение (Belief) → DBN → фильтр частиц → POMCP-дерево → FSM-исполнитель.


---

## Требования

| Зависимость | Версия |
|---|---|
| JDK | 21 |
| Gradle (системный) | любой (используется только для генерации wrapper) |
| Gradle (wrapper) | 8.5 |
| [rcrs-server](https://github.com/roborescue/rcrs-server) | master |
| [adf-core-java](https://github.com/roborescue/adf-core-java) | master |

---

## Подготовка зависимостей

### adf-core-java

```bash
git clone https://github.com/roborescue/adf-core-java.git
cd adf-core-java
./gradlew publishToMavenLocal
cd ..
```

Устанавливается как `com.github.roborescue:adf-core:master-SNAPSHOT` —
обратите внимание: artifactId именно `adf-core`, не `adf-core-java`.

### rcrs-server

В `build.gradle` репозитория используется устаревший API Gradle (`classifier`
вместо `archiveClassifier`). При использовании Gradle 8+ сборка падает.

**Костыль:** патч перед сборкой:

```bash
git clone https://github.com/roborescue/rcrs-server.git
cd rcrs-server
sed -i 's/\.classifier = /\.archiveClassifier = /g' build.gradle
./gradlew publishToMavenLocal
cd ..
```

`publishToMavenLocal` выполняется **один раз**.
Повторять только после `git pull` или переустановки системы.

---

## Сборка

```bash
# Шаг 1: сгенерировать Gradle Wrapper (один раз)
# Системный Gradle 4.4.1 не совместим с JDK 21.
# Генерируем wrapper с нужной версией вручную:

mkdir -p gradle/wrapper
cat > gradle/wrapper/gradle-wrapper.properties << 'PROPS'
distributionBase=GRADLE_USER_HOME
distributionPath=wrapper/dists
distributionUrl=https\://services.gradle.org/distributions/gradle-8.5-bin.zip
zipStoreBase=GRADLE_USER_HOME
zipStorePath=wrapper/dists
PROPS

curl -o gradlew \
  https://raw.githubusercontent.com/gradle/gradle/v7.6.0/gradlew
chmod +x gradlew

curl -o gradle/wrapper/gradle-wrapper.jar \
  https://raw.githubusercontent.com/gradle/gradle/v7.6.0/gradle/wrapper/gradle-wrapper.jar

# Шаг 2: собрать проект
./gradlew build
```

---

## Запуск

```bash
# Шаг 1: создать config/ в корне (ADF ищет module.cfg здесь, не в src/)
mkdir -p config
cp src/main/resources/config/module.cfg config/module.cfg

# Шаг 2: дать права на запуск скрипту
chmod +x launch.sh

# Шаг 3: запустить симулятор (в отдельном терминале)
cd /path/to/rcrs-server/scripts
bash start-comprun.sh -m ../maps/test/map -c ../maps/test/config

# Шаг 4: запустить агентов (после сообщения "Waiting for agents")
cd /path/to/bayesian_agent
./launch.sh -all
```

Успешный запуск выглядит так:
```
[FINISH] Done connecting to server (3 agents)
[AMBULANCE    482151809] initialized
[FIRE         210552869] initialized
[POLICE       1962675462] initialized
[AMBULANCE    482151809] tick 1 → action=AgentAction{REST}
[POLICE       1962675462] tick 2 → action=AgentAction{CLEAR, target=...}
```

