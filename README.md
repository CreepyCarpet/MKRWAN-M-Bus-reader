<h1>LoRaWAN enabled M-Bus-reader firmware for the Arduino MKR WAN 1310</h1>

Attempts to OTAA-join the connected TTS instance on power-on. Scans M-Bus addresses for devices and sends full M-Bus telegrams binary-encoded at chosen interval. Further development will enable an application to choose the content of the uplink packages. Package interval should also be automatically updates based on spreading factor. 

<h2>Todo</h2>
<ol>
    <li>ADR handling<ul>
        <li>Tune sending interval depending on package length and spreading factor<ul>
            <li><s>Package time calculator</s></li>
        </ul></li>
        <li><s>Set minimum interval</s></li>
        <li><s>Set specific interval</s></li>
    </ul></li>
    <li>Local M-Bus parsing<ul>
        <li>Parse to cayenne</li>
    </ul></li>
    <li>Downlink handling<ul>
        <li>Get devices</li>
        <li>Enable/Disable devices</li>
        <li>Set uplink interval</li>
        <li>Custom uplinks selection<ul> 
            <li>Choose specific datatypes/sensors to send as uplink</li>
            <li>Request custom uplink list</li>
            <li>Per device specific packages</li>
            </ul></li>
        <li>Power cycle command</li>
        <li><s>Random join wait - Use deviceEUI as seed</s></li>
        <li><s>Increasing rejoin interval</s></li>
        <li>Rescan M-Bus addresses command</li>
        <li><s>Forced rejoin</s></li>
    </ul></li>
</ol>