/*jslint */
/*global AdobeEdge: false, window: false, document: false, console:false, alert: false */
(function (compId) {

    "use strict";
    var im='images/',
        aud='media/',
        vid='media/',
        js='js/',
        fonts = {
            'Digital-7': '<link rel=\"stylesheet\" href=\"fonts.css\" type=\"text/css\" media=\"screen\" title=\"\" charset=\"utf-8\" />',
            '\'Digital tech\'': '<link rel=\"stylesheet\" href=\"fonts.css\" type=\"text/css\" media=\"screen\" title=\"\" charset=\"utf-8\" />',
            'MagistralC': '<link rel=\"stylesheet\" href=\"fonts.css\" type=\"text/css\" media=\"screen\" title=\"\" charset=\"utf-8\" />'        },
        opts = {
            'gAudioPreloadPreference': 'auto',
            'gVideoPreloadPreference': 'auto'
        },
        resources = [
        ],
        scripts = [
            js+"jquery-2.0.3.min.js",
            js+"jquery.ui.touch-punch.min.js",
            js+"jquery-ui.min.js"
        ],
        symbols = {
            "stage": {
                version: "6.0.0",
                minimumCompatibleVersion: "5.0.0",
                build: "6.0.0.400",
                scaleToFit: "both",
                centerStage: "horizontal",
                resizeInstances: false,
                content: {
                    dom: [
                        {
                            id: 'intergace',
                            type: 'image',
                            rect: ['86px', '0px', '1450px', '1080px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"intergace.png",'0px','0px']
                        },
                        {
                            id: 'Text',
                            type: 'group',
                            rect: ['346px', '111px', '807px', '173px', 'auto', 'auto'],
                            c: [
                            {
                                id: 'ROBOT08',
                                type: 'image',
                                rect: ['0px', '0px', '160px', '160px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"RobotR.png",'0px','0px']
                            },
                            {
                                id: 'CASE_DEFINITION_GREEN',
                                type: 'image',
                                rect: ['136px', '1px', '745px', '160px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_GREEN1.png",'0px','0px']
                            },
                            {
                                id: 'TextIN',
                                type: 'text',
                                rect: ['172px', '30px', '685px', '99px', 'auto', 'auto'],
                                opacity: '1',
                                text: "<p style=\"margin: 0px;\"><span style=\"font-size: 36px;\">​</span><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">The intelligent brick is the brain of a robot. It is able to run all your programs</span></p>",
                                align: "left",
                                font: ['Digital-7', [34, "px"], "rgba(0,0,0,1)", "500", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'presentBrick',
                            type: 'video',
                            tag: 'video',
                            rect: ['100px', '272px', '1189px', '669px', 'auto', 'auto'],
                            clip: 'rect(26.666748046875px 1037.888671875px 629px 228.888671875px)',
                            source: [vid+"presentBrick.webm"],
                            preload: 'auto'
                        },
                        {
                            id: 'Text1',
                            type: 'group',
                            rect: ['847px', '150px', '1172', '477', 'auto', 'auto'],
                            c: [
                            {
                                id: 'titreCopy4',
                                type: 'text',
                                rect: ['162px', '139px', '452px', '89px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">The Brick Buttons</p>",
                                align: "center",
                                font: ['Arial, Helvetica, sans-serif', [40, "px"], "rgba(3,55,154,1.00)", "700", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'Text4Copy',
                                type: 'text',
                                rect: ['162px', '101px', '483px', '391px', 'auto', 'auto'],
                                text: "<p style=\"margin:0px\">​<span style=\"font-weight: 900; color: rgb(255, 183, 0);\">​</span></p><p style=\"margin:0px\"><span style=\"font-weight: 900; color: rgb(255, 183, 0);\">​​</span></p><p style=\"margin: 0px;\"><span style=\"font-weight: 900; color: rgb(255, 183, 0);\">​1.&nbsp;</span><span style=\"font-weight: 900; color: rgb(236, 155, 2);\">Central</span></p><p style=\"margin:0px\">​Press the center button to select and confirm various operations.</p><p style=\"margin: 0px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\">​<span style=\"font-weight: 900; color: rgb(236, 155, 2);\">2. Back</span></p><p style=\"margin:0px\">​<span style=\"color: rgb(236, 155, 2); font-weight: 900;\">​</span>Used to cancel actions, interrupt a program in progress and turn off the EV3.2 brick</p><p style=\"margin: 0px;\"><span style=\"color: rgb(236, 155, 2); font-weight: 900;\">3. Left, Right, Up,Down</span></p><p style=\"margin: 0px;\"></p><p style=\"margin: 0px;\">These four buttons allow you to navigate in the content of the EV3 brick​</p><p style=\"margin:0px\">​</p>",
                                align: "left",
                                font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [34, "px"], "rgba(0,0,0,1)", "500", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'Text2',
                            type: 'group',
                            rect: ['874px', '204px', '1172', '477', 'auto', 'auto'],
                            c: [
                            {
                                id: 'titre',
                                type: 'text',
                                rect: ['155px', '133px', '452px', '89px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">Input Ports</p>",
                                align: "center",
                                font: ['Arial, Helvetica, sans-serif', [40, "px"], "rgba(3,55,154,1.00)", "700", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'Text4',
                                type: 'text',
                                rect: ['213px', '195px', '405px', '306px', 'auto', 'auto'],
                                text: "<p style=\"margin:0px\">​<span style=\"font-weight: 800;\">Ports 1,2,3 and 4</span> are used to connect the sensors.</p><p style=\"margin: 0px;\">​</p><p style=\"margin:0px\">​</p>",
                                align: "left",
                                font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [34, "px"], "rgba(0,0,0,1)", "500", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'Text3',
                            type: 'group',
                            rect: ['443px', '299px', '1171', '361', 'auto', 'auto'],
                            c: [
                            {
                                id: 'text',
                                type: 'text',
                                rect: ['643px', '99px', '434px', '165px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​<span style=\"font-weight: 800;\">Ports A, B, C and D </span>are used to connect the motors.</p><p style=\"margin: 0px;\">​</p><p style=\"margin: 0px;\">​</p>",
                                align: "left",
                                font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [32, "px"], "rgba(0,0,0,1)", "500", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'titreCopy',
                                type: 'text',
                                rect: ['592px', '37px', '452px', '89px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">Output Ports</p>",
                                align: "center",
                                font: ['Arial, Helvetica, sans-serif', [40, "px"], "rgba(3,55,154,1.00)", "700", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'Text7',
                            type: 'group',
                            rect: ['1005px', '173px', '1119', '361', 'auto', 'auto'],
                            c: [
                            {
                                id: 'textee',
                                type: 'text',
                                rect: ['112px', '225px', '459px', '165px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">USB port + SD card port​</p>",
                                align: "left",
                                font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [32, "px"], "rgba(0,0,0,1)", "500", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'titreCopy2',
                                type: 'text',
                                rect: ['36px', '166px', '452px', '89px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​Connections</p>",
                                align: "center",
                                font: ['Arial, Helvetica, sans-serif', [40, "px"], "rgba(3,55,154,1.00)", "700", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'Text9',
                            type: 'group',
                            rect: ['421px', '183px', '1128', '361', 'auto', 'auto'],
                            c: [
                            {
                                id: 'tesxteee',
                                type: 'text',
                                rect: ['687px', '218px', '395px', '165px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\"></p><p style=\"margin: 0px;\">&nbsp;The Ev3 brick is equipped with a Loudspeaker allowing you to emit sounds​</p>",
                                align: "left",
                                font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [32, "px"], "rgba(0,0,0,1)", "500", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'titreCopy3',
                                type: 'text',
                                rect: ['584px', '152px', '452px', '89px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">Loudspeaker</p>",
                                align: "center",
                                font: ['Arial, Helvetica, sans-serif', [40, "px"], "rgba(3,55,154,1.00)", "700", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'Text10',
                            type: 'group',
                            rect: ['513px', '202px', '1128', '361', 'auto', 'auto'],
                            c: [
                            {
                                id: 'tesxteeeCopy',
                                type: 'text',
                                rect: ['614px', '204px', '364px', '165px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​For powering the EV3 Brick, you can use<span style=\"font-weight: 800;\"> AA batteries</span> or the <span style=\"font-weight: 800;\">EV3 rechargeable battery</span></p>",
                                align: "left",
                                font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [32, "px"], "rgba(0,0,0,1)", "500", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'titreCopy5',
                                type: 'text',
                                rect: ['523px', '142px', '452px', '89px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">Power supply</p>",
                                align: "center",
                                font: ['Arial, Helvetica, sans-serif', [40, "px"], "rgba(3,55,154,1.00)", "700", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: '_1',
                            type: 'image',
                            rect: ['172px', '246px', '105px', '80px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"12.png",'0px','0px']
                        },
                        {
                            id: 'b1',
                            type: 'image',
                            rect: ['172px', '246px', '105px', '80px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"b12.png",'0px','0px']
                        },
                        {
                            id: 'pointer',
                            type: 'image',
                            rect: ['258px', '288px', '34px', '42px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"pointer.png",'0px','0px']
                        },
                        {
                            id: '_2',
                            type: 'image',
                            rect: ['172px', '354px', '105px', '80px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"22.png",'0px','0px']
                        },
                        {
                            id: 'b2',
                            type: 'image',
                            rect: ['172px', '354px', '105px', '80px', 'auto', 'auto'],
                            cursor: 'default',
                            fill: ["rgba(0,0,0,0)",im+"b22.png",'0px','0px']
                        },
                        {
                            id: '_2Copy',
                            type: 'image',
                            rect: ['172px', '354px', '105px', '80px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"22.png",'0px','0px']
                        },
                        {
                            id: '_3',
                            type: 'image',
                            rect: ['172px', '465px', '105px', '80px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"32.png",'0px','0px']
                        },
                        {
                            id: 'b3',
                            type: 'image',
                            rect: ['172px', '465px', '105px', '80px', 'auto', 'auto'],
                            cursor: 'default',
                            fill: ["rgba(0,0,0,0)",im+"b32.png",'0px','0px']
                        },
                        {
                            id: '_3Copy',
                            type: 'image',
                            rect: ['172px', '465px', '105px', '80px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"32.png",'0px','0px']
                        },
                        {
                            id: '_4',
                            type: 'image',
                            rect: ['172px', '578px', '105px', '80px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"42.png",'0px','0px']
                        },
                        {
                            id: 'b4',
                            type: 'image',
                            rect: ['172px', '578px', '105px', '80px', 'auto', 'auto'],
                            cursor: 'default',
                            fill: ["rgba(0,0,0,0)",im+"b42.png",'0px','0px']
                        },
                        {
                            id: '_4Copy',
                            type: 'image',
                            rect: ['172px', '578px', '105px', '80px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"42.png",'0px','0px']
                        },
                        {
                            id: '_5',
                            type: 'image',
                            rect: ['172px', '691px', '105px', '80px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"52.png",'0px','0px']
                        },
                        {
                            id: 'b5',
                            type: 'image',
                            rect: ['172px', '691px', '105px', '80px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"b52.png",'0px','0px']
                        },
                        {
                            id: '_5copy',
                            type: 'image',
                            rect: ['172px', '691px', '105px', '80px', 'auto', 'auto'],
                            cursor: 'default',
                            fill: ["rgba(0,0,0,0)",im+"52.png",'0px','0px']
                        },
                        {
                            id: '_6',
                            type: 'image',
                            rect: ['172px', '797px', '105px', '80px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"62.png",'0px','0px']
                        },
                        {
                            id: 'b6',
                            type: 'image',
                            rect: ['172px', '797px', '105px', '80px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"b62.png",'0px','0px']
                        },
                        {
                            id: '_6copy',
                            type: 'image',
                            rect: ['172px', '797px', '105px', '80px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"62.png",'0px','0px']
                        },
                        {
                            id: 'TITRE',
                            symbolName: 'TITRE_1',
                            type: 'rect',
                            rect: ['470px', '36px', '390', '57', 'auto', 'auto']
                        }
                    ],
                    style: {
                        '${Stage}': {
                            isStage: true,
                            rect: ['null', 'null', '1650px', '1080', 'auto', 'auto'],
                            overflow: 'hidden',
                            fill: ["rgba(240,240,240,0)"]
                        }
                    }
                },
                timeline: {
                    duration: 5500,
                    autoPlay: false,
                    data: [

                    ]
                }
            },
            "TITRE": {
                version: "6.0.0",
                minimumCompatibleVersion: "5.0.0",
                build: "6.0.0.400",
                scaleToFit: "none",
                centerStage: "none",
                resizeInstances: false,
                content: {
                    dom: [
                        {
                            type: 'text',
                            font: ['Digital-7', [146, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            id: 'Text6Copy2',
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(0, 61, 65); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-family: \'Digital tech\'; font-size: 62px; text-transform: uppercase; font-weight: 500;\">The intelligent brick</span></p>',
                            clip: 'rect(0px 661.60986328125px 57px 0px)',
                            rect: ['27px', '-11px', '699px', '57px', 'auto', 'auto']
                        },
                        {
                            type: 'text',
                            font: ['Digital-7', [146, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            id: 'Text6',
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-family: \'Digital tech\'; font-size: 62px; text-transform: uppercase; font-weight: 500;\">The intelligent brick</span></p>',
                            clip: 'rect(0px 661.60986328125px 57px 0px)',
                            rect: ['27px', '-11px', '699px', '57px', 'auto', 'auto']
                        }
                    ],
                    style: {
                        '${symbolSelector}': {
                            rect: [null, null, '390px', '57px']
                        }
                    }
                },
                timeline: {
                    duration: 5000,
                    autoPlay: true,
                    data: [
                        [
                            "eid132",
                            "clip",
                            0,
                            500,
                            "linear",
                            "${Text6}",
                            [0,0,57,0],
                            [0,60,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid112",
                            "clip",
                            500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,60,57,0],
                            [0,100,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid114",
                            "clip",
                            750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,100,57,0],
                            [0,140,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid116",
                            "clip",
                            1000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,140,57,0],
                            [0,160,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid118",
                            "clip",
                            1250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,160,57,0],
                            [0,190,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid120",
                            "clip",
                            1500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,190,57,0],
                            [0,220,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid122",
                            "clip",
                            1750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,220,57,0],
                            [0,260,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid124",
                            "clip",
                            2000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,260,57,0],
                            [0,270,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid126",
                            "clip",
                            2250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,270,57,0],
                            [0,290,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid128",
                            "clip",
                            2500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,290,57,0],
                            [0,320,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid130",
                            "clip",
                            2750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,320,57,0],
                            [0,350,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid157",
                            "clip",
                            3000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,350,57,0],
                            [0,380,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid159",
                            "clip",
                            3250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,380,57,0],
                            [0,405,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid161",
                            "clip",
                            3500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,405,57,0],
                            [0,475,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid165",
                            "clip",
                            3750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,475,57,0],
                            [0,515,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid167",
                            "clip",
                            4000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,515,57,0],
                            [0,530,57,-155.55541992188],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid173",
                            "clip",
                            4250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,530,57,-155.555419921875],
                            [0,555,57,-155.55541992188],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid175",
                            "clip",
                            4500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,555,57,-155.555419921875],
                            [0,595,57,-155.55541992188],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid177",
                            "clip",
                            4750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,595,57,-155.555419921875],
                            [0,650,57,-155.55541992188],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ]
                    ]
                }
            },
            "TITRE_1": {
                version: "6.0.0",
                minimumCompatibleVersion: "5.0.0",
                build: "6.0.0.400",
                scaleToFit: "none",
                centerStage: "none",
                resizeInstances: false,
                content: {
                    dom: [
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            font: ['Digital-7', [146, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            id: 'Text6Copy2',
                            rect: ['27px', '-11px', '699px', '57px', 'auto', 'auto'],
                            clip: 'rect(0px 661.60986328125px 57px 0px)',
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(0, 61, 65); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-family: \'Digital tech\'; font-size: 62px; text-transform: uppercase; font-weight: 500;\">The intelligent brick</span></p>'
                        },
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            font: ['Digital-7', [146, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            id: 'Text6',
                            rect: ['27px', '-11px', '699px', '57px', 'auto', 'auto'],
                            clip: 'rect(0px 661.60986328125px 57px 0px)',
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-family: \'Digital tech\'; font-size: 62px; text-transform: uppercase; font-weight: 500;\">The intelligent brick</span></p>'
                        }
                    ],
                    style: {
                        '${symbolSelector}': {
                            rect: [null, null, '390px', '57px']
                        }
                    }
                },
                timeline: {
                    duration: 5250,
                    autoPlay: true,
                    data: [
                        [
                            "eid132",
                            "clip",
                            0,
                            500,
                            "linear",
                            "${Text6}",
                            [0,0,57,0],
                            [0,60,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid112",
                            "clip",
                            500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,60,57,0],
                            [0,100,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid114",
                            "clip",
                            750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,100,57,0],
                            [0,140,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid116",
                            "clip",
                            1000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,140,57,0],
                            [0,160,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid118",
                            "clip",
                            1250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,160,57,0],
                            [0,190,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid120",
                            "clip",
                            1500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,190,57,0],
                            [0,220,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid122",
                            "clip",
                            1750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,220,57,0],
                            [0,260,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid124",
                            "clip",
                            2000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,260,57,0],
                            [0,270,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid126",
                            "clip",
                            2250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,270,57,0],
                            [0,290,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid128",
                            "clip",
                            2500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,290,57,0],
                            [0,320,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid130",
                            "clip",
                            2750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,320,57,0],
                            [0,350,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid157",
                            "clip",
                            3000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,350,57,0],
                            [0,380,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid159",
                            "clip",
                            3250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,380,57,0],
                            [0,405,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid161",
                            "clip",
                            3500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,405,57,0],
                            [0,475,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid165",
                            "clip",
                            3750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,475,57,0],
                            [0,505,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid167",
                            "clip",
                            4000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,505,57,0],
                            [0,530,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid199",
                            "clip",
                            4250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,530,57,0],
                            [0,540,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid201",
                            "clip",
                            4500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,540,57,0],
                            [0,565,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid203",
                            "clip",
                            4750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,565,57,0],
                            [0,595,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid205",
                            "clip",
                            5000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,595,57,0],
                            [0,680.7080078125,57,-78.28271484375],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ]
                    ]
                }
            }
        };

    AdobeEdge.registerCompositionDefn(compId, symbols, fonts, scripts, resources, opts);

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0200_Brique_edgeActions.js");
})("EDGE-6689876");
