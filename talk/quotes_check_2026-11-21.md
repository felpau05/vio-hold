# Slide 1 quote check (rows 1a, 2a, 2b, 4b of business_sources.md)

Checked on 2026-11-21. Each page was downloaded with curl at 03:16 EST and searched as raw page text (no summarising tool).
Result: all four quotes are **same** as the page text. Notes below each row say what a judge could still question.

| Row | Exact sentence on the page | vs table quote | Publisher | Date on page | Opened (EST) |
|---|---|---|---|---|---|
| 1a | "The number of global positioning system (GPS) signal loss events increased by 220% between 2021 and 2024 according to IATA's data from the Global Aviation Data Management Flight Data eXchange (GADM FDX)." | same | EASA newsroom, joint EASA and IATA press release "EASA and IATA outline comprehensive plan to mitigate GNSS interference risks" | 18 Jun 2025 (dateline "COLOGNE, June 18, 2025") | 03:16 |
| 2a | "Ukrainian UAV losses remain at approximately 10,000 per month." | same | RUSI special report "Meatgrinder: Russian Tactics in the Second Year of Its Invasion of Ukraine" (Watling and Reynolds) | 19 May 2023 | 03:16 |
| 2b | "Russian electronic warfare (EW) remains potent, with an approximate distribution of at least one major system covering each 10 km of front." | same (table quote is the opening clause) | RUSI, same report | 19 May 2023 | 03:16 |
| 4b | "Maintain rangefinding/geolocation capability during Global Navigation Satellite System (GNSS) degradation; provide a fallback navigation (e.g., visual odometry, inertial fusion, Real-Time Kinematic correction)." and "Development funding: this challenge will offer $2.1 million of development funding (total), with funding focussing on promising concepts that can quickly advance their capabilities." | same | National Defence, IDEaS challenge "True north precision: Low cost drones with laser ranging" | Page details 2026-06-11; "This CFP is closed and is no longer accepting proposals." | 03:16 |

## Notes for the talk

- **1a.** The 220% sentence is a quote from the IATA Senior Vice President inside the press release, and the number is IATA's flight-data count (GADM FDX). Say "IATA data, published with EASA". The old URL in the table redirects to a new address: https://www.easa.europa.eu/en/newsroom-and-events/press-releases/easa-and-iata-outline-comprehensive-plan-mitigate-gnss
- **2a.** The report is from May 2023. Say "RUSI estimated in 2023". Do not present 10,000 a month as a current number.
- **2a and 2b.** The page puts these in one paragraph. The sentence between them is: "These systems are heavily weighted towards the defeat of UAVs and tend not to try and deconflict their effects." The page still does not say that EW caused the 10,000 losses. "In the same paragraph, RUSI says Russian EW is aimed mostly at defeating drones" is supported. "10,000 drones lost to jamming" is not.
- **4b.** The challenge is about rangefinding for small ISR drones. Fallback navigation is one listed outcome (EO3), not the main subject. A second line (DO7) also names visual odometry: "Autonomous or semi-autonomous (e.g. waypoint navigation) operation in GNSS-denied environments using multiple alternative navigation methods (e.g., visual odometry, inertial fusion)." The call is closed.

## Second check: rows 3c and 6a (question bank)

| Row | Exact sentence on the page | vs table quote | Publisher | Date on page | Opened (EST) |
|---|---|---|---|---|---|
| 6a | "When inspecting the underside of a bridge deck, the aircraft is likely to lose GPS connection because of the overhead obstruction and the low penetrating power of GPS signals." (Section 2.1.2, "Current UAS Technologies") | same (table quote is the middle clause) | National Academies of Sciences, Engineering, and Medicine, NCHRP Research Report 1114 "Uncrewed Aerial Systems Applications for Bridge Inspections: Element-Level Bridge Data Collection", chapter 2 "Research Approach" | 2024 (citation on the page) | 03:18 |

- **6a.** The next sentence on the page supports a position-hold argument directly: "Losing a GPS signal may eliminate or reduce the airframe's ability to hold a certain position and altitude, requiring the remote pilot to manually control the aircraft or rely on other sensors (ultrasonic proximity sensors or first-person view cameras) for stability."

| Row | Result |
|---|---|
| 3c | **Not checked on the live page.** https://www.modalai.com/products/starling-2 returned HTTP 429 (rate limit) at 03:18 and again at 03:20; the product-data address (`starling-2.js`) also returned HTTP 429. |

What the archived copy shows (Internet Archive, captured 2026-02-15 06:05 UTC: http://web.archive.org/web/20260215060503/https://www.modalai.com/products/starling-2, opened 03:19 EST):
- Comparison block: "Starling 2 / Price: from $2,949.99" and "Starling 2 Max / Price: from $2,999.99".
- Starling 2 variants run from $2,849.99 (drone only, no radios) to $3,899.99 (2 batteries, controller and case).
- **$3,199.99 does not appear** in this copy. Prices on this page change over time, so the table's figure may be newer, or it may be wrong.
- Image sensors in this copy: Starling 2 has "Triple AR0144 fisheye CV image sensor"; Starling 2 Max has "Dual AR0144 fisheye CV image sensor". Row 5 of business_sources.md says Starling 2 has dual AR0144: that matches the Max, not the Starling 2.

Advice: do not say an exact price. "A commercial visual-navigation development drone (ModalAI Starling 2) costs about $3,000" is true of both the archived copy and the table. Open the live page in a normal browser before the talk if an exact number is needed.

## Page URLs as opened

- 1a: https://www.easa.europa.eu/en/newsroom-and-events/press-releases/easa-and-iata-publish-comprehensive-plan-mitigate-risks-gnss (redirects to the URL above)
- 2a, 2b: https://www.rusi.org/explore-our-research/publications/special-resources/meatgrinder-russian-tactics-second-year-its-invasion-ukraine (the summary page; the full report PDF, 11 MB, was not opened)
- 4b: https://www.canada.ca/en/department-national-defence/programs/defence-ideas/element/competitive-projects/challenges/true-north-precision.html
