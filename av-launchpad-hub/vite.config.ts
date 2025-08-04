import react from "@vitejs/plugin-react-swc";
import { componentTagger } from "lovable-tagger";
import path from "path";
// Remove the problematic rollup-plugin-node-polyfills
// import nodePolyfills from "rollup-plugin-node-polyfills";
import { fileURLToPath, URL } from "url";
import { defineConfig } from "vite";

// Establish __dirname for ESM
const __dirname = fileURLToPath(new URL(".", import.meta.url));

export default defineConfig(({ mode }) => ({
  server: {
    host: "::",
    port: 8080,
    hmr: { overlay: false },
  },
  plugins: [
    react(), // React w/ SWC
    // Use Vite's built-in polyfill handling instead
    mode === "development" && componentTagger(),
  ].filter(Boolean),
  resolve: {
    alias: [
      // Root alias
      {
        find: "@",
        replacement: fileURLToPath(new URL("./src", import.meta.url)),
      },
      // Comprehensive draco3d stubbing - catch all variations
      {
        find: /^draco3d$/,
        replacement: path.resolve(__dirname, "src/draco-stub.js"),
      },
      {
        find: "draco3d/draco3d.js",
        replacement: path.resolve(__dirname, "src/draco-stub.js"),
      },
      {
        find: "draco3d/draco_decoder.wasm",
        replacement: path.resolve(__dirname, "src/draco-stub.js"),
      },
      {
        find: "draco3d/draco_encoder_nodejs.js",
        replacement: path.resolve(__dirname, "src/empty.js"),
      },
      // Catch WASM plugin's placeholder import "a"
      { find: /^a$/, replacement: path.resolve(__dirname, "src/empty.js") },
      // Stub out fs
      { find: /^fs$/, replacement: path.resolve(__dirname, "src/empty.js") },
      // Node core module fallbacks
      { find: /^path$/, replacement: "path-browserify" },
      // Add setImmediate polyfill
      { find: /^process$/, replacement: "process/browser.js" },
      {
        find: /^setImmediate$/,
        replacement: path.resolve(__dirname, "src/setImmediate-polyfill.js"),
      },
    ],
  },
  define: {
    "process.env": {},
    "process.env.NODE_ENV": JSON.stringify(process.env.NODE_ENV),
    global: "globalThis",
  },
  optimizeDeps: {
    exclude: ["draco3d", "draco3d/draco3d.js", "draco3d/draco_decoder.wasm"],
    include: [
      "@foxglove/regl-worldview",
      "path-browserify",
      "process/browser.js",
      "rosbag",
    ],
  },
  build: {
    rollupOptions: {
      external: (id) => {
        // Mark draco3d WASM files as external
        if (id.includes("draco3d") && id.includes(".wasm")) {
          return true;
        }
        return false;
      },
    },
  },
}));
