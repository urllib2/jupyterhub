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
                scaleToFit: "height",
                centerStage: "horizontal",
                resizeInstances: false,
                content: {
                    dom: [
                        {
                            id: 'intergace',
                            type: 'image',
                            rect: ['80px', '0px', '1450px', '1080px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"intergace.png",'0px','0px']
                        },
                        {
                            id: 'TITRE',
                            symbolName: 'TITRE',
                            type: 'rect',
                            rect: ['534px', '32px', '390', '57', 'auto', 'auto']
                        },
                        {
                            id: 'person',
                            type: 'image',
                            rect: ['614px', '442px', '398px', '501px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"person2.png",'0px','0px']
                        },
                        {
                            id: 'personUmberlla',
                            type: 'image',
                            rect: ['614px', '442px', '398px', '501px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"personUmberlla.png",'0px','0px']
                        },
                        {
                            id: 'person2',
                            type: 'image',
                            rect: ['614px', '442px', '398px', '501px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"person.png",'0px','0px']
                        },
                        {
                            id: 'Sun',
                            type: 'image',
                            rect: ['729px', '155px', '325px', '291px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"Sun.png",'0px','0px']
                        },
                        {
                            id: 'rain',
                            type: 'image',
                            rect: ['505px', '70px', '640px', '640px', 'auto', 'auto'],
                            clip: 'rect(0px 640px 551.111083984375px 0px)',
                            fill: ["rgba(0,0,0,0)",im+"rain.png",'0px','0px'],
                            transform: [[],[],[],['0.8375','0.75636']]
                        },
                        {
                            id: 'button2',
                            type: 'group',
                            rect: ['283px', '296px', '197', '86', 'auto', 'auto'],
                            cursor: 'pointer',
                            c: [
                            {
                                id: 'RoundRect',
                                type: 'rect',
                                rect: ['0px', '-12px', '191px', '80px', 'auto', 'auto'],
                                borderRadius: ["10px", "10px", "10px", "10px"],
                                opacity: '1',
                                fill: ["rgba(252,252,252,1.00)"],
                                stroke: [3,"rgb(0, 0, 0)","solid"]
                            },
                            {
                                id: 'Text',
                                type: 'text',
                                rect: ['18px', '8px', '162px', '65px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px; text-align: center;\">​<span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">Good</span></p>",
                                font: ['Arial, Helvetica, sans-serif', [30, "px"], "rgba(0,0,0,1)", "normal", "none", "", "break-word", "normal"]
                            }]
                        },
                        {
                            id: 'button',
                            type: 'group',
                            rect: ['1118px', '284px', '197', '86', 'auto', 'auto'],
                            cursor: 'pointer',
                            c: [
                            {
                                id: 'RoundRect2',
                                type: 'rect',
                                rect: ['0px', '0px', '191px', '80px', 'auto', 'auto'],
                                borderRadius: ["10px", "10px", "10px", "10px"],
                                fill: ["rgba(252,252,252,1.00)"],
                                stroke: [3,"rgb(0, 0, 0)","solid"]
                            },
                            {
                                id: 'Text2',
                                type: 'text',
                                rect: ['65px', '17px', '109px', '57px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">Bad</span></p>",
                                align: "left",
                                font: ['Arial, Helvetica, sans-serif', [30, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'Text3',
                            type: 'text',
                            rect: ['193px', '501px', '364px', '186px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px; text-align: left;\">The man appreciates the heat and the light and enjoys an ice cream.​</p>",
                            align: "center",
                            font: ['Arial, Helvetica, sans-serif', [30, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "41px", "", "none"]
                        },
                        {
                            id: 'Text4',
                            type: 'text',
                            rect: ['1061px', '501px', '364px', '291px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\">​The man opens his umbrella for protection. e protéger.</p>",
                            align: "left",
                            font: ['Arial, Helvetica, sans-serif', [30, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "", "", "none"]
                        },
                        {
                            id: 'cursorclick',
                            type: 'image',
                            rect: ['310px', '370px', '129px', '122px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"cursorclick.gif",'0px','0px']
                        },
                        {
                            id: 'Text5',
                            type: 'text',
                            rect: ['667px', '114px', '254px', '57px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px; text-align: center;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 500;\">If the weather&nbsp;</span></p><p style=\"margin: 0px; text-align: center;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 500;\">​is</span></p>",
                            align: "left",
                            font: ['Arial, Helvetica, sans-serif', [30, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "", "", "none"]
                        },
                        {
                            id: 'Ellipse',
                            type: 'ellipse',
                            rect: ['671px', '102px', '231px', '82px', 'auto', 'auto'],
                            borderRadius: ["50%", "50%", "50%", "50%"],
                            fill: ["rgba(192,192,192,0)"],
                            stroke: [2,"rgb(0, 0, 0)","solid"]
                        },
                        {
                            id: 'Rectangle',
                            type: 'rect',
                            rect: ['906px', '143px', '292px', '0px', 'auto', 'auto'],
                            fill: ["rgba(192,192,192,0)"],
                            stroke: [2,"rgb(0, 0, 0)","solid"]
                        },
                        {
                            id: 'RectangleCopy',
                            type: 'rect',
                            rect: ['378px', '145px', '289px', '0px', 'auto', 'auto'],
                            fill: ["rgba(192,192,192,0)"],
                            stroke: [2,"rgb(0, 0, 0)","solid"]
                        },
                        {
                            id: 'Rectangle2',
                            type: 'rect',
                            rect: ['1198px', '143px', '0px', '137px', 'auto', 'auto'],
                            fill: ["rgba(192,192,192,0)"],
                            stroke: [2,"rgb(0, 0, 0)","solid"]
                        },
                        {
                            id: 'Rectangle2Copy',
                            type: 'rect',
                            rect: ['378px', '145px', '0px', '135px', 'auto', 'auto'],
                            fill: ["rgba(192,192,192,0)"],
                            stroke: [2,"rgb(0, 0, 0)","solid"]
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
                    duration: 5250,
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
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            font: ['Digital-7', [146, 'px'], 'rgba(1,66,65,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            rect: ['67px', '-4px', '655px', '57px', 'auto', 'auto'],
                            id: 'Text6Copy2',
                            clip: 'rect(0px 657.021728515625px 57px 0px)',
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"color: rgb(0, 61, 65); font-size: 62px; font-family: \'Digital tech\'; font-weight: 500;\">the selection</span></p>'
                        },
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            font: ['Digital-7', [146, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            rect: ['67px', '-4px', '699px', '57px', 'auto', 'auto'],
                            id: 'Text6',
                            clip: 'rect(0px 636px 57px 0px)',
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\">​<span style=\"font-size: 62px; font-family: \'Digital tech\'; font-weight: 500;\">the selection</span></p>'
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
                            [0,636,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ]
                    ]
                }
            }
        };

    AdobeEdge.registerCompositionDefn(compId, symbols, fonts, scripts, resources, opts);

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0210_AnalogieSelection_edgeActions.js");
})("EDGE-6689876");
