import { exec } from "child_process";
import { existsSync, mkdir, writeFile } from "fs";

import config from "./ros-ts-generator-config";

const BUILD_DIR = "build";
const CONFIG_FILENAME = "ros-ts-generator-config.json";
const CONFIG_FILEPATH = `${BUILD_DIR}/${CONFIG_FILENAME}`;

function main() {
    if (!existsSync(BUILD_DIR)) {
        mkdir(BUILD_DIR, {}, (err) => {
            if (err) {
                console.error(err);
            }
        });
    }

    writeFile(CONFIG_FILEPATH, JSON.stringify(config, null, 4), (err) => {
        if (err) {
            console.error(err);
        }
    });

    exec(
        `npx ros-typescript-generator --config ${CONFIG_FILEPATH}`,
        (error, stdout, stderr) => {
            if (error) {
                console.log(`error: ${error.message}`);
                return;
            }
            if (stderr) {
                console.log(`stderr: ${stderr}`);
                return;
            }
            console.log(`stdout: ${stdout}`);
        }
    );
}

main();
