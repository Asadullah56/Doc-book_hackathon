import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import useBaseUrl from '@docusaurus/useBaseUrl';
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
    link: '/module-1/ros2-communication-backbone',
  },
  {
    title: 'Module 2: Digital Twin',
    description: (
      <>
        Explore Gazebo physics simulation, Unity visualization, and environment modeling for robot development.
      </>
    ),
    link: 'module-2/chapter-1-gazebo-physics',
  },
  {
    title: 'Module 3: AI Brain',
    description: (
      <>
        Leverage NVIDIA Isaac for perception, VSLAM for navigation, and advanced computer vision systems.
      </>
    ),
    link: 'module-3/chapter-1-photorealistic-intelligence',
  },
  {
    title: 'Module 4: VLA Capstone',
    description: (
      <>
        Integrate LLMs, OpenAI Whisper, and autonomous action systems for complete humanoid intelligence.
      </>
    ),
    link: 'module-4/chapter-1-voice-to-action',
  },
];

function Feature({title, description, link}: FeatureItem) {
  return (  
    <div className={clsx('col', styles.featureCardCol)}>
      <div className={clsx('card', styles.featureCard)}>
        <div className={clsx('card__body', styles.cardBody)}>
          <Heading as="h3" className={styles.cardTitle}>{title}</Heading>
          <p className={styles.cardDescription}>{description}</p>
        </div>
        <div className={clsx('card__footer', styles.cardFooter)}>
          <a className={clsx('button', styles.learnMoreButton)} href={link}>
            Learn More
          </a>
        </div>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  const moduleBgUrl = useBaseUrl('/img/module-bg.jpg');

  return (
    <section
      className={styles.features}
      style={{
        backgroundImage: `linear-gradient(rgba(15, 23, 42, 0.9), rgba(15, 23, 42, 0.9)), url('${moduleBgUrl}')`,
        backgroundSize: 'cover',
        backgroundPosition: 'center',
        backgroundAttachment: 'scroll',
        marginBottom: '0',
        paddingBottom: '0'
      }}
    >
      <div className="container padding-vert--xl">
        <div className="text--center padding-top--lg">
          <Heading as="h2" style={{ color: 'var(--bot-cyan)' }}>The 4 Modules of Humanoid Academy</Heading>
          <p className="hero__subtitle" style={{ color: '#9ca3af' }}>Master each foundational element of autonomous humanoid development</p>
        </div>
        <div className={clsx('row', styles.featureGrid)}>
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
