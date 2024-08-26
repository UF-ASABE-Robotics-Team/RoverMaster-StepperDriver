# RoverMaster Distributed Driver Firmware

### Useful Commands

+ **Build project**

    ```sh
    pio run -e [A|B]
    ```

+ **Generate compile_commands.json** (for clangd)

    ```sh
    pio run --target compiledb -e [A|B]
    ```

+ **Compile and Upload**

    ```sh
    pio run --target upload -e [A|B]
    ```

+ **Clean build files**

    ```sh
    pio run --target clean
    ```
