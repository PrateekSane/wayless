const webpack = require("webpack");

module.exports = {
  webpack: {
    configure: (config) => {
      // Stub Node core modules
      config.resolve.fallback = {
        fs: false,
        path: require.resolve("path-browserify"),
        process: require.resolve("process/browser"),
      };

      // Provide process globally
      config.plugins.push(
        new webpack.ProvidePlugin({
          process: "process/browser",
        })
      );

      return config;
    },
  },
};
