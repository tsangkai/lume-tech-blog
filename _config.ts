import lume from "lume/mod.ts";
import plugins from "./plugins.ts";

import figures from "npm:markdown-it-image-figures";

// Set the markdown plugins
const markdown = {
  plugins: [[figures, { figcaption: "alt" }]],
  keepDefaultPlugins: true,
};

const site = lume({
  src: "./src",
}, { markdown });

site.use(plugins());

export default site;
