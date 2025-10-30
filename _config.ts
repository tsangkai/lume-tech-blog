import lume from "lume/mod.ts";
import plugins from "./plugins.ts";

import figures from "npm:markdown-it-image-figures";

const site = lume(
  {
    src: "./src",
  },
  {
    markdown: {
      plugins: [
        [figures, { figcaption: "alt" }],
      ],
      keepDefaultPlugins: true,
    },
  },
);

site.use(plugins());

export default site;
