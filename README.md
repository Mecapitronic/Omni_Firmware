# Mecapi Firmware Holonome

Firmware du Robot Holonome pour [ESP32-S3-Tiny](https://www.waveshare.com/wiki/ESP32-S3-Tiny).
Le firmware est compilé avec [Platform.io](https://platformio.org/).

### Compilation

```bash
pio run --environment Esp32_NO_WIFI
pio run --environment Esp32_WOKWI
pio run --environment Esp32_WITH_WIFI
```

## Tasks

Tasks can be assigned priorities of 0 to 24 (configMAX_PRIORITIES - 1).  Zero is the lowest priority.

## Résumé des Tasks sur Core 1

| Task Name                  | Core | Description                                             | Priority |
|----------------------------|------|---------------------------------------------------------|----------|
| Main Loop (deleted)        | 1    | (Task deleted)                                          | 1        |
| Lidar TaskLidar            | 1    | Read Serial Lidar and save incoming obstacle            | 10       |
| Main timerMotionCallback   | 1    | Update Position, Trajectory, Motion, Motors (non-block) | MAX      |
| Main TaskUpdate            | 1    | Update HMI, ring, servo position                        | 15       |
| Main TaskHandleCommand     | 1    | Handle command                                          | 5        |
| Main TaskMatch             | 1    | State machine of the Match (strategy code)              | 10       |

## Résumé des Tasks sur Core 0

| Task Name                  | Core | Description                                             | Priority |
|----------------------------|------|---------------------------------------------------------|----------|
| ESP32_Helper Update        | 0    | Read Serial Debug and save incoming command into queue  | 5        |
| Wifi_Helper Update         | 0    | Wifi/server/UDP reconnect, read client, handle OTA      | 10       |
| Main TaskTeleplot          | 0    | Print over Serial / wifi server / wifi UDP              | 10       |

## Commandes console

Envoyer `Help` pour afficher la légende et les aides enregistrées par
Esp32_Helper, y compris les aides propres au robot. Ces dernières sont
enregistrées sans gestionnaire pour conserver leur traitement par la tâche
`TaskHandleCommand`. Les commandes disponibles dans les bibliothèques dépendent
des modules initialisés et de la variante compilée.

La convention commune est documentée dans
[Esp32_Helper](lib/Esp32_Helper/README.md#aide-et-syntaxe-des-commandes) :

```text
Commande:<nom:type@unite>(:nom:type@unite=defaut) - Description
```

`<...>` est obligatoire ; `(...)` est facultatif, séparateur `:` compris.
Les types sont `int` signé 32 bits, `string`, `bool` transmis avec `0` ou `1`,
et `int|string`.
Les unités convenues sont `ms`, `s`, `%`, `deg`, `mm`, `mm/s`, `bit/s`, `byte`.
L'unité et la valeur par défaut sont omises lorsqu'elles ne s'appliquent pas.
Les contraintes sont documentaires, sans validation automatique supplémentaire.
