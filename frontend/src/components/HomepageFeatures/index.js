import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'ROS 2 Framework',
    Svg: require('@site/static/img/ros-icon.svg').default,
    description: (
      <>
        Build on the Robot Operating System 2 (ROS 2) framework for robust communication,
        hardware abstraction, and device drivers for your robotic applications.
      </>
    ),
  },
  {
    title: 'Advanced Simulation',
    Svg: require('@site/static/img/simulation-icon.svg').default,
    description: (
      <>
        Develop and test your robotics algorithms in high-fidelity simulation
        environments before deploying to real hardware.
      </>
    ),
  },
  {
    title: 'Generative AI Integration',
    Svg: require('@site/static/img/genai-icon.svg').default,
    description: (
      <>
        Leverage cutting-edge generative AI models to enhance your robotics
        capabilities with intelligent decision-making and adaptive behaviors.
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
