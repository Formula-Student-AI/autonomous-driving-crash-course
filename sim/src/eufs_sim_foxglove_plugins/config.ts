module.exports = {
    webpack: (config) => {
      config.devtool = undefined; // disable source maps to reduce bundle size
      return config;
    },
  };