<h1>LoRaWAN enabled M-Bus-reader firmware for the Arduino MKRWAN 1310</h1>

Attempts to OTAA-join the connected TTS instance on power-on. Scans M-Bus addresses for devices and sends full M-Bus telegrams binary-encoded at chosen interval.  
Further development will enable an application to choose the content of the uplink packages. Package interval should also be automatically updated based on spreading factor and requested intervals.  

<h2>MKRWAN 1310 pinout</h2>
<img title="MKRWAN 1310 pinout" alt="MKRWAN 1310 pinout" src="./ABX00029-pinout.png">

<h2>Todo</h2>
<ol>
    <li>ADR handling<ul>
        <li><s>Tune sending interval depending on package length and spreading factor</s><ul>
            <li><s>Package time calculator</s></li>
        </ul></li>
        <li><s>Set minimum interval</s></li>
        <li><s>Set specific interval</s></li>
        <li>Implement ADR functionality</li>
        <li>Check for ADR req on downlinks and adjust datarate accordingly</li>
        <li>Rebuild interval database with new ADR</li>
    </ul></li>
    <li><s>Heartbeat function</s></li>
    <li>Time keeping, time based events</li>
    <li>Local M-Bus parsing<ul>
        <li>Parse to cayenne</li>
    </ul></li>
    <li>Downlink handling<ul>
        <li><s>Downlink switch statement</s></li>
        <li><s>Get devices</s></li>
        <li><s>Enable/Disable devices</s></li>
        <li><s>Set uplink interval</s></li>
        <li><s>Custom uplinks selection</s><ul> 
            <li><s>Choose specific datatypes/sensors to send as uplink</s></li>
            <li>Request custom uplink list</li>
            <li><s>Per device specific packages</s></li>
            <li>Option for event based packages<ul>
                <li>Delta before event?</li>
                <li>Specific members are event-based</li>
                </ul></li>
            </ul></li>
        <li><s>Power cycle command</s><ul>
            </ul></li>
        <li><s>Random join wait - Use deviceEUI as seed</s></li>
        <li><s>Increasing rejoin interval</s></li>
        <li><s>Rescan M-Bus addresses command</s></li>
        <li><s>Modify M-Bus scan command to only add new devices, perhaps throw the old list?</s></li>
        <li><s>Forced rejoin</s></li>
    </ul></li>
    <li>Network optimization<ul>
        <li>Prime number intervals?</li>
        <li>Improved ALOHA</li>
    </ul></li>
</ol>