# Event site and event-day conditions (research, 2026-11-15)

Confidence key: HIGH = a source states it. LOW = the inference (marked "INFERENCE").

## 0. Most important findings

1. **The Grand Final date is 2026-11-21, not 22. Time is 5:30 - 8:00 PM EST.** Venue: Canadian War Museum, Ottawa. Source: https://redteamhack.ca/ and https://redteamhack.ca/winners
2. **The sun sets at about 4:26 PM on 22 Nov (7:11 AM sunrise).** The listed slot (5:30 - 8:00 PM) is after sunset. The page describes pitches and an awards ceremony. It does not say when or where the flight happens. If the flight is in the listed slot, it is in the dark, and a visible-light camera over a dark park would see almost nothing. This must be confirmed with the organizers. (INFERENCE: the flight may be earlier in the day. No source says.)
3. The organizer's page gives only one line about the challenge: "Live position hold without GNSS using Visual-Inertial Odometry". It gives no hardware, limits, altitude, or judging details for the flight.
4. Late-November climate: mean wind 13.8 km/h (3.8 m/s) from the west at 10 m. Daily high +5 C, low -2 C. Snow on the ground on a few days of the month (see section 3). Frost and fog are possible.

## 1. Wind

Source (primary): Environment and Climate Change Canada normals, station OTTAWA MACDONALD-CARTIER INT'L A (CYOW), November, period **1981-2010**. Read through the ECCC open-data API:
https://api.weather.gc.ca/collections/climate-normals/items?CLIMATE_IDENTIFIER=6106000&MONTH=11&f=json&limit=100
(Station list: https://api.weather.gc.ca/collections/climate-stations/items?q=OTTAWA&f=json&limit=20 . CYOW has identifier 6106000 in the normals. The API lists "OTTAWA INTL A" 6106001 with no normals.)

Measured values (anemometer height is 10 m by WMO standard. The API did not state it.):
- Mean of hourly wind speed: **13.78 km/h** (3.83 m/s).
- Most frequent wind direction: **270 degrees true (west)**. The Wikipedia Ottawa page also says "Prevailing winds are generally from the West" (https://en.wikipedia.org/wiki/Ottawa).
- Days per November with at least one hourly speed above: 10 km/h 29.1 days; 20 km/h 17.3; 30 km/h 6.83; 40 km/h 1.93; 50 km/h 0.23.
- Days with at least one hourly speed <= 5 km/h: 17.8 days (so calm hours are common).
- Days with "Wind >= 28 knots" (52 km/h): 1.12 days. Days with wind >= 34 knots (63 km/h): 0.23 days. (The API does not say whether this is gust or sustained. Unknown.)
- Another site (currentresults.com, 1981-2010, from ECCC) gives 13.8 km/h and 1.1 days per November of 52 km/h or more: https://www.currentresults.com/Weather/Canada/Cities/wind-november.php . It matches the API.
- Gust statistics: **not found**. No gust number is given.
- Direction spread beyond "west most frequent": not found.

### Wind at height (ESTIMATES, not measurements)
Reference: u10 = 13.78 km/h mean. Two standard rules:
- Log law: u(z) = u10 * ln(z/z0) / ln(10/z0). Roughness z0 = 0.03 m (short grass/open), 0.1 m (low crops/parkland), 0.5 m (suburban/ scattered buildings).
- Power law: u(z) = u10 * (z/10)^alpha. alpha = 0.14 (open, neutral), 0.2 (mixed), 0.3 (urban/stable).

| Height | Log z0=0.03 | Log z0=0.1 | Log z0=0.5 | Power 0.14 | Power 0.2 | Power 0.3 |
|---|---|---|---|---|---|---|
| 50 m | 17.6 km/h | 18.6 | 21.2 | 17.3 | 19.0 | 22.3 |
| 100 m | 19.2 | 20.7 | 24.4 | 19.0 | 21.8 | 27.5 |
| 120 m | 19.7 | 21.2 | 25.2 | 19.5 | 22.7 | 29.0 |

Plain reading (INFERENCE, low confidence): typical mean wind at 100-120 m is about 19-25 km/h (5-7 m/s) for parkland roughness. Real gusts on a day with a 30-50 km/h hourly peak could reach 60+ km/h at height. The profile rules assume neutral air and flat terrain. In late-November evening air the layer is often stable, which gives stronger shear (alpha larger). The river valley and the museum building change local flow. These numbers are not a forecast.

## 2. Temperature and cold effects

Primary (ECCC, CYOW, 1981-2010, November): mean daily max **5.36 C**, mean daily min **-2.42 C**, mean **1.47 C**. Days with max <= 0 C: 4.97. Days with min <= 0 C: 20.63. Days with min < -10 C: 1.93. Days with wind chill < -20: 0.57. Days with freezing rain or drizzle: 2.07. Days with fog/ice fog/freezing fog: 2.9. Days with blowing snow: 0.57. Source: the API URL in section 1.
Newer period (Wikipedia Ottawa article, 1991-2020 airport table): mean daily max 5.8 C, min -2.4 C, snowfall 20.7 cm, 6.5 snowy days. https://en.wikipedia.org/wiki/Ottawa (secondary source.)
Other readings of the same normals: relative humidity at 06:00 LST 83.5 percent, at 15:00 LST 68.1 percent. Hours with 8-10 tenths cloud: 473.5 of about 720 (66 percent). Bright sunshine 92.4 hours in November, 32 percent of daylight hours. Mean station pressure 100.27 kPa.

Effects of cold on the drone (general, from hobby and aviation guides; not from Orqa):
- Battery: cold lithium-polymer packs lose capacity and show more voltage sag. Sources give 30-50 percent capacity loss at very low temperatures and a danger of sudden sag below about 10 C. Pre-warm packs, avoid hard throttle bursts at the start, land with a larger reserve. Numbers differ by source and some sources sell batteries. Refs: https://hobbyking.com/blog/LiPoBatteryLowTemperaturePerformanceAnalysis , https://www.grepow.com/blog/fly-a-drone-in-winter-with-low-temperature-battery.html , https://open-exam-prep.com/study-guides/faa-part-107/chapter4/battery-performance
- Barometer drift: Did not find a source for the barometer of the Orqa flight controller. Barometric height drifts with temperature change and with weather pressure change (INFERENCE, low confidence: a cold-soaked board that warms in flight will show a slow height bias. The method uses baro height for scale, so a bias gives a proportional scale error in metres).
- Lens fog and frost: not found in an authoritative source. INFERENCE: an analog camera moved from a warm room to cold air, or flying through humid air (06:00 humidity 83 percent), may fog or frost. Plan a lens warm-up and a dry cloth.

## 3. Snow cover on the ground

Primary (ECCC normals, **Ottawa CDA** station 6105976, 1981-2010, November): https://api.weather.gc.ca/collections/climate-normals/items?CLIMATE_IDENTIFIER=6105976&MONTH=11&f=json&limit=100
- Mean daily snow depth 1.14 cm. Median daily snow depth 0.67 cm.
- Mean month-end snow depth 3 cm.
- Days in November with depth >= 1 cm: 4.92. >= 5 cm: 2.77. >= 10 cm: 1.35. >= 20 cm: 0.19. (of 30 days)

CYOW itself (same period, same API pattern, identifier 6106000): mean daily depth 1.53 cm, median 0.72 cm, month-end 3.83 cm. Days with depth >= 1 cm: 6; >= 5 cm: 3.3; >= 10 cm: 1.6; >= 20 cm: 0.5. Snowfall 20.22 cm total in November (std dev 21.41, so the month varies a lot).

Reading (INFERENCE): about 16-20 percent of November days have 1 cm or more of snow on the ground. Those days cluster late in the month, so on 21-22 Nov the chance is probably higher than the month average. Found no day-specific probability and no "median first lasting snow cover" date. Those are unknowns. Median depth below 1 cm means that most years have bare or thinly dusted ground on average days. A thin dusting still changes the texture (white, low contrast), and a 5-10 cm cover would remove texture on lawns.

For comparison, white Christmas (>= 2 cm on 25 Dec) in Ottawa is 79 percent (1955-2024), 72 percent (2000-2024): https://www.canada.ca/en/environment-climate-change/services/weather-general-tools-resources/historical-christmas-snowfall-data.html . That is a month later and not the question, but it shows the trend toward lasting cover starts in December.

## 4. Light (Ottawa, 22 Nov 2026)

Source: https://www.timeanddate.com/sun/canada/ottawa?month=11&year=2026 (secondary, but computed astronomy).
- Sunrise 7:11 AM (azimuth 118 degrees). Sunset 4:26 PM (azimuth 241 degrees). Day length 9 h 15 min. (Local time, EST, after DST ended 1 Nov.)
- Solar noon 11:48 AM, sun altitude **24.4 degrees**.
- Shadow length at solar noon = object height / tan(24.4 deg) = about 2.2 times object height (the calculation). A 24 m museum peak casts roughly a 53 m shadow at noon, longer toward the evening. The sun is low all day: shadows are long and move fast in the afternoon (INFERENCE: moving shadows are a real error source for a feature tracker on a flat lawn).
- Nov 21 is 1-2 minutes different from the 22 Nov figures (INFERENCE from the 2 min 5 s per-day change in the same source).
- ECCC normals, CYOW November: 66 percent of hours with 8-10 tenths cloud (overcast), which gives soft, shadow-free light but low light level; 32 percent of daylight hours with bright sunshine.
- Civil twilight timing: not looked up. After sunset (4:26 PM), ambient light is low within about 30-40 minutes (INFERENCE). Urban lights in LeBreton are not described by any source found.

## 5. Airspace

Primary and official sources:
- CYR537: Class F restricted airspace over Parliament Hill, circle radius 0.4 mile, centre N45 25 29.05 W075 41 56.74, surface to 3000 ft, continuous. Controlling agency RCMP. Aircraft including model aircraft may not operate there without authorization. Effective 17 June 2021. Source: https://www.navcanada.ca/en/flight-planning/service-notices/2021-06-17-ottawa.aspx
- Distance (the calculation, rough): the museum (about 45.417 N, 75.717 W; from public maps, not retrieved from a source here) is about 2 km west-southwest of the CYR537 centre. 0.4 mile is 0.64 km. So the museum is outside the circle by roughly 1.3 km. INFERENCE, low confidence. Verify on a chart.
- Parliament Hill drone process: https://pps.parl.ca/wp-content/uploads/2025/03/ProcedureDronesCYR537-b-Updated-.pdf (search snippet only; not read in full).
- CYOW control zone surface to 2000 ft MSL with Class C terminal area above (from Canada Flight Supplement snippets via search, not fully verified). CYRO (Rockcliffe) and Gatineau CYND are separate nearby zones. Exact boundaries over LeBreton are NOT confirmed. Could not read the NAV CANADA drone tool: https://nrc.canada.ca/en/drone-tool-2/map.html returned HTTP 403.
- Transport Canada: general limit 122 m (400 ft) AGL; controlled airspace needs an advanced pilot and NAV CANADA permission; basic ops stay in uncontrolled airspace. Since 1 April 2025 an SFOC is required to fly microdrones at advertised events (an outdoor event promoted to the public). SFOC fees from 4 Nov 2025: $20 very low, $75 low, $900 medium, $2,000 high. Source: https://tc.canada.ca/en/aviation/drone-safety/2025-summary-changes-canada-drone-regulations ; also https://tc.canada.ca/en/aviation/drone-safety/drone-pilot-licensing/get-permission-special-drone-operations/higher-risk-environments
- Plausible event height (INFERENCE, low): the hard legal ceiling is 122 m AGL in all cases except a special authorization. Organizers must hold permissions for downtown-Ottawa airspace and likely limit height far below that, or use a net/enclosure. The brief says "as high as possible", so the cap is probably an event-set number. No source gives it. Ask the organizers. Plan the sim for 20-120 m, with the best case near 100 m or less.

## 6. Ground the camera sees (descriptions only; no imagery retrieved)

From https://en.wikipedia.org/wiki/Canadian_War_Museum and the search results:
- Museum: 1 Vimy Place, LeBreton Flats, south bank of the Ottawa River. Site about 7.5 ha. Highest point 24 m.
- **Green roof**: self-seeding, 20,500 m2, blends into parkland on the west side. Sloped concrete on the east side. Late-November colour: dormant grass (INFERENCE).
- Roads bound the east and south of the site. The Capital Pathway runs along the site. The south bank of the river is the west and north boundary.
- "The Commons": an urban park directly south, used for museum events (open lawn and hard surface; INFERENCE: events may put tents/temporary structures there).
- Ottawa River to the north and west (water: no texture, moving, glare; fatal for tracking if it fills the view).
- Pimisi station on O-Train Line 1 (LRT). Rail and platform give straight, repeating lines.
- LeBreton Flats: formerly industrial, decontaminated. Redevelopment is ongoing: arena parcel sale (4.5 ha), Aqueduct District, library parcel; timelines unfixed (https://ncc-ccn.gc.ca/projects/lebreton-major-event-centre , https://www.ctvnews.ca/ottawa/article/ncc-suggests-new-sens-arena-at-lebreton-flats-could-open-by-mid-2030s-in-developer-pitch/). So there may be bare-earth lots, fences and construction. No source found gives the current state.
- Parking: not found in a source.
Reading for the simulator (INFERENCE): expect large low-texture areas (lawn, water, asphalt, snow) with sparse high-contrast features (path edges, curbs, roof edges, rail lines). The real-photo simulator at the event site is the right test. Add runs with a river-filled view and with a uniform snow-covered lawn.

## 7. The event itself (public pages)

Organizer site https://redteamhack.ca/ (fetched):
- Series name "2026 Red Team Hackathon Series". Organizers: Remote Robotic Systems and Orqa. Marketing sponsor The Icebreaker. Sponsors: Remote Robotic Systems, Orqa, Calian, ThinkOn.
- **Grand Final: 21 Nov 2026, Canadian War Museum, 5:30 - 8:00 PM EST.** Format: finalists pitch live for the national title; winners crowned at an awards ceremony. Tickets via Eventbrite (did not find the Eventbrite page).
- Ottawa Final challenge: "Contested Battlefield Aerial Operations": "Live position hold without GNSS using Visual-Inertial Odometry". Nothing more. No altitude, hardware, camera, limit, or judging text for the flight.
- Series schedule listed: 7 Mar Vancouver (Find The Force, UBC); 21 Mar Calgary (Voice-Driven UxS); 30 May Ottawa (Verified Canadian Supply Chains, Bayview Yards); 20 Jun Toronto (Comms-Denied Autonomy). The winners page lists "Comms-Denied Autonomy" at Bayview Yards, Ottawa, 2 May, and the Toronto one on 20 June. The two pages disagree on names and dates. https://redteamhack.ca/winners
- Grand Final qualifiers: the first place of each series event qualified (per the organizer's winners page). The list of teams is not kept here.
- Terms https://redteamhack.ca/terms: max 11 teams; judging criteria Entrepreneurial Drive, Algorithm, Technical Implementation, Feasibility & Scalability, Storytelling & Vision (weighted, weights not given); prizes 1st $10,000, 2nd $6,000, 3rd-5th $1,000 each, Best Presenter $1,000 (these appear to be the regional-event terms; the page also says $200K+ for the series, and the BCIT article says a $20,000 prize pool); bring own laptops/hardware; liability waiver may be required; the terms do not mention drones.
- BCIT story (Red Team Hackathon Series, organizers include the CEO of Remote Robotic Systems): https://www.bcit.ca/news/stories/computing-takes-first-place/ . Listing for Find The Force (software-only, labeled data): https://allhackathons.com/hackathon/find-the-force-redteam-hackathon-series/
- Orqa: press about the MRM2-10AI (Eurosatory, June 2026) says it has an "Auto Pilot Board" with vision-based terminal guidance for GNSS-denied operation, onboard compute for customer AI models, an open development architecture and a Developer Program at developer.orqafpv.com. No processor or flight controller named. https://soldiersystems.net/2026/06/11/orqa-unveils-the-mrm2-10ai-at-eurosatory-a-tactical-drone-designed-to-dominate-in-the-age-of-electronic-warfare/ . The developer page fetched with no content. The brief's "M2-AI" may be this product (INFERENCE; the names differ). The team's i.MX8M Plus + STM32H743 and 230400 baud notes are NOT confirmed by any source found.

## 8. Unknowns

- Flight time of day and place on 21 Nov, and whether it is indoors, outdoors, or at night.
- Flight altitude cap; the limits (arrive 72 h before).
- Gust statistics and a day-specific snow probability for 21-22 Nov; the median date of first lasting snow cover.
- Exact airspace class/altitude limit at the museum; NAV CANADA tool was not readable.
- Barometer model of the Orqa board and its temperature drift.
- Current state of the LeBreton ground (construction, snow clearing, event tents). No imagery fetched.
- 1991-2020 wind and snow normals (the API held only 1981-2010 for these).
