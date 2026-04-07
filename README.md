# bayesian_agent

Минимальный скелет команды спасателей для **RoboCup Rescue Agent Simulation (RCRS)**,
построенный поверх [adf-core-java](https://github.com/roborescue/adf-core-java).

Включает все три подразделения: **санитары**, **пожарные**, **полиция**.
Каждый агент реализован на трёх заменяемых модулях.
Текущие реализации — намеренно примитивные заглушки, служащие baseline-ом
и точками подключения байесовских компонентов.

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

---

## Известные проблемы и костыли

### 1. Системный Gradle 4.4.1 несовместим с JDK 21

Ubuntu может поставить очень старый Gradle. Решение — Gradle Wrapper 8.5
(см. раздел Сборка). Gradle 7.x тоже не подходит — не поддерживает JDK 21
(`Unsupported class file major version 65`).

### 2. rcrs-server/build.gradle использует устаревший API

`classifier = 'sources'` → `archiveClassifier = 'sources'` (убрали в Gradle 8).
Решение: `sed -i 's/\.classifier = /\.archiveClassifier = /g' build.gradle`
перед `publishToMavenLocal`.

### 3. artifactId adf-core-java — это adf-core

При `publishToMavenLocal` репозиторий `adf-core-java` публикуется под
artifactId `adf-core` (не `adf-core-java`). В `build.gradle` нашего агента
указывать именно `adf-core`.

### 4. google-collect:0.5 и jsi:1.0.0 недоступны в Maven

`google-collect:0.5` не существует в Maven Central — исключён, заменён на
`guava:31.1-jre` (прямой наследник с совместимым API).

`jsi:1.0.0` нужен для WorldModel агентов, но не публикуется в Maven.
Подключён как локальный файл из `~/rcrs-server/lib/jsi-1.0.0.jar`.

### 5. Пакеты ADF переименованы в версии 4.x

Все классы переехали под `adf.core.*`:

| Старый пакет | Новый пакет |
|---|---|
| `adf.agent.info.*` | `adf.core.agent.info.*` |
| `adf.agent.action.*` | `adf.core.agent.action.*` |
| `adf.agent.communication.*` | `adf.core.agent.communication.*` |
| `adf.agent.develop.*` | `adf.core.agent.develop.*` |
| `adf.agent.module.*` | `adf.core.agent.module.*` |
| `adf.agent.precompute.*` | `adf.core.agent.precompute.*` |
| `adf.component.tactics.*` | `adf.core.component.tactics.*` |

### 6. think() возвращает Action, а не void

В ADF 4.x `Tactics.think()` возвращает `Action`. Центры управления
(`TacticsCenter`) по-прежнему используют `void`.

### 7. Центры управления не имеют метода precompute()

`TacticsCenter` (родитель AmbulanceCentre, FireStation, PoliceOffice) не имеет
`precompute()` в отличие от `Tactics` (родитель полевых агентов).

### 8. Platoon.postConnect() содержит захардкоженное устаревшее имя класса

Баг в adf-core-java: в `Platoon.postConnect()` дефолтное значение для
`ChannelSubscriber` захардкожено как `adf.component.communication.ChannelSubscriber`
(старый пакет, не существует). Решение: явно прописать в `module.cfg`:

```properties
MessageManager.PlatoonChannelSubscriber  : adf.impl.module.comm.DefaultChannelSubscriber
MessageManager.PlatoonMessageCoordinator : adf.impl.module.comm.DefaultMessageCoordinator
```

### 9. module.cfg нужен в двух местах

ADF ищет конфиг по пути `config/module.cfg` относительно `user.dir` (корня
проекта). Файл в `src/main/resources/config/` нужен для сборки, но не
используется при запуске через `launch.sh`. Нужно поддерживать обе копии
синхронизированными.

### 10. checkIsCarrying() — заглушка

`agentInfo.someoneOnBoard()` не существует в ADF 4.x. Метод проверки наличия
жертвы в машине санитара реализован как заглушка (всегда `false`). Нужно
уточнить реальное имя метода:

```bash
grep -r "isLoaded\|someoneOnBoard\|loadedHuman" ~/adf-core-java/src/
```

