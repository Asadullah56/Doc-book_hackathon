import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading' ;
import Head from '@docusaurus/Head';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={clsx(styles.heroGrid, "flex flex-col md:flex-row items-center justify-between gap-10")}>
          <div className={clsx(styles.heroText, "flex-1")}>
            <div className={styles.handbookBadge}>
              Handbook
            </div>
            <Heading as="h1" className={clsx("hero__title", styles.fadeIn)}>
              Humanoid Academy
            </Heading>
            <p className={clsx("hero__subtitle", styles.fadeIn)}>A Practical Handbook on Physical AI and Humanoid Robotics</p>
            <div className={clsx("hero__description", styles.fadeIn)}>
              The ultimate technical manual for building the future. Master fundamental <span className={clsx(styles.highlightKeyword)}>ROS 2</span> foundations, integrate advanced <span className={clsx(styles.highlightKeyword)}>AI brains</span>, and orchestrate complex <span className={clsx(styles.highlightKeyword)}>humanoid systems</span> through a structured 4-module <span className={clsx(styles.typewriterWord)}>journey<span className={styles.cursor}>|</span></span>.
            </div>
            <div className={styles.buttons}>
              <Link
                className={clsx("button button--lg", styles.heroButton)}
                to="/intro">
                Explore the Library
              </Link>
            </div>
          </div>
          <div className={clsx(styles.heroImage, "flex-1 max-w-[350px] md:max-w-[500px] w-full h-auto object-contain")}>
            <img
              src="/img/robot-hero.jpg"
              alt="Robot Hero"
              className={clsx(styles.floatingImage, styles.pulseImage, "w-full h-auto object-contain")}
            />
          </div>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout>
      <Head>
        <title>Humanoid Academy - {siteConfig.tagline}</title>
        <meta name="description" content="Mastering Physical AI: From ROS 2 Foundations to Autonomous Humanoid Systems" />
      </Head>
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
