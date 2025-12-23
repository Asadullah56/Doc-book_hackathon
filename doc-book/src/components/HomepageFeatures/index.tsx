import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  description: ReactNode;
  link: string;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Module 1: Nervous System',
    description: (
      <>
        Master ROS 2 fundamentals, node communication, and distributed systems architecture for humanoid robotics.
      </>
    ),
    link: '/docs/module-1/chapter-1-the-ros-2-communication-backbone',
  },
  {
    title: 'Module 2: Digital Twin',
    description: (
      <>
        Explore Gazebo physics simulation, Unity visualization, and environment modeling for robot development.
      </>
    ),
    link: '/docs/module-2/chapter-1-gazebo-physics-simulation',
  },
  {
    title: 'Module 3: AI Brain',
    description: (
      <>
        Leverage NVIDIA Isaac for perception, VSLAM for navigation, and advanced computer vision systems.
      </>
    ),
    link: '/docs/module-3/chapter-1-photorealistic-intelligence',
  },
  {
    title: 'Module 4: VLA Capstone',
    description: (
      <>
        Integrate LLMs, OpenAI Whisper, and autonomous action systems for complete humanoid intelligence.
      </>
    ),
    link: '/docs/module-4/chapter-1-voice-to-action',
  },
];

function Feature({title, description, link}: FeatureItem) {
  return (
    <div className={clsx('col col--6')}>
      <div className="card">
        <div className="card__body text--center padding-horiz--md">
          <Heading as="h3">{title}</Heading>
          <p>{description}</p>
        </div>
        <div className="card__footer text--center">
          <a className="button button--secondary button--sm" href={link}>
            Learn More
          </a>
        </div>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="text--center padding-top--lg">
          <Heading as="h2">The 4 Modules of Humanoid Academy</Heading>
          <p className="hero__subtitle">Master each foundational element of autonomous humanoid development</p>
        </div>
        <div className="row padding-top--lg">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
