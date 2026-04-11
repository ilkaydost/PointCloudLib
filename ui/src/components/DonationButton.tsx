import styles from './css/DonationButton.module.css';

const DONATION_URL = 'https://github.com/sponsors/ilkaydost?frequency=one-time&sponsor=ilkaydost';

export function DonationButton(): JSX.Element {
  return (
    <a
      href={DONATION_URL}
      target="_blank"
      rel="noopener noreferrer"
      className={styles.button}
      title="Support server costs"
    >
      <svg
        className={styles.icon}
        viewBox="0 0 24 24"
        fill="currentColor"
        aria-hidden="true"
        xmlns="http://www.w3.org/2000/svg"
      >
        <path d="M2 21V19H20V21H2ZM20 8V5H18V8H20ZM20 3C20.5523 3 21 3.44772 21 4V9C21 9.55228 20.5523 10 20 10H18V12C18 14.2091 16.2091 16 14 16H8C5.79086 16 4 14.2091 4 12V4C4 3.44772 4.44772 3 5 3H20ZM16 10V8H6V12C6 13.1046 6.89543 14 8 14H14C15.1046 14 16 13.1046 16 12V10ZM6 6V4H16V6H6Z" />
      </svg>
      Help me cover the domain costs
    </a>
  );
}
