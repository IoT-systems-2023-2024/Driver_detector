SamacSys ECAD Model
569577/1190528/2.50/4/4/Crystal or Oscillator

DESIGNSPARK_INTERMEDIATE_ASCII

(asciiHeader
	(fileUnits MM)
)
(library Library_1
	(padStyleDef "r90_80"
		(holeDiam 0)
		(padShape (layerNumRef 1) (padShapeType Rect)  (shapeWidth 0.800) (shapeHeight 0.900))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 0) (shapeHeight 0))
	)
	(textStyleDef "Default"
		(font
			(fontType Stroke)
			(fontFace "Helvetica")
			(fontHeight 50 mils)
			(strokeWidth 5 mils)
		)
	)
	(patternDef "CX2016DB32000D0FLJCC" (originalName "CX2016DB32000D0FLJCC")
		(multiLayer
			(pad (padNum 1) (padStyleRef r90_80) (pt -0.700, -0.550) (rotation 90))
			(pad (padNum 2) (padStyleRef r90_80) (pt 0.700, -0.550) (rotation 90))
			(pad (padNum 3) (padStyleRef r90_80) (pt 0.700, 0.550) (rotation 90))
			(pad (padNum 4) (padStyleRef r90_80) (pt -0.700, 0.550) (rotation 90))
		)
		(layerContents (layerNumRef 18)
			(attr "RefDes" "RefDes" (pt 0.000, 0.000) (textStyleRef "Default") (isVisible True))
		)
		(layerContents (layerNumRef 28)
			(line (pt -1 0.8) (pt 1 0.8) (width 0.2))
		)
		(layerContents (layerNumRef 28)
			(line (pt 1 0.8) (pt 1 -0.8) (width 0.2))
		)
		(layerContents (layerNumRef 28)
			(line (pt 1 -0.8) (pt -1 -0.8) (width 0.2))
		)
		(layerContents (layerNumRef 28)
			(line (pt -1 -0.8) (pt -1 0.8) (width 0.2))
		)
		(layerContents (layerNumRef 30)
			(line (pt -2.15 1.95) (pt 2.15 1.95) (width 0.1))
		)
		(layerContents (layerNumRef 30)
			(line (pt 2.15 1.95) (pt 2.15 -1.95) (width 0.1))
		)
		(layerContents (layerNumRef 30)
			(line (pt 2.15 -1.95) (pt -2.15 -1.95) (width 0.1))
		)
		(layerContents (layerNumRef 30)
			(line (pt -2.15 -1.95) (pt -2.15 1.95) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt -0.6 -1.3) (pt -0.6 -1.3) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(arc (pt -0.7, -1.3) (radius 0.1) (startAngle .0) (sweepAngle 180.0) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt -0.8 -1.3) (pt -0.8 -1.3) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(arc (pt -0.7, -1.3) (radius 0.1) (startAngle 180.0) (sweepAngle 180.0) (width 0.2))
		)
	)
	(symbolDef "CX2016DB32000D0FLJCC" (originalName "CX2016DB32000D0FLJCC")

		(pin (pinNum 1) (pt 0 mils -100 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -125 mils) (rotation 0]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 2) (pt 1200 mils -100 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 970 mils -125 mils) (rotation 0]) (justify "Right") (textStyleRef "Default"))
		))
		(pin (pinNum 3) (pt 1200 mils 0 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 970 mils -25 mils) (rotation 0]) (justify "Right") (textStyleRef "Default"))
		))
		(pin (pinNum 4) (pt 0 mils 0 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -25 mils) (rotation 0]) (justify "Left") (textStyleRef "Default"))
		))
		(line (pt 200 mils 100 mils) (pt 1000 mils 100 mils) (width 6 mils))
		(line (pt 1000 mils 100 mils) (pt 1000 mils -200 mils) (width 6 mils))
		(line (pt 1000 mils -200 mils) (pt 200 mils -200 mils) (width 6 mils))
		(line (pt 200 mils -200 mils) (pt 200 mils 100 mils) (width 6 mils))
		(attr "RefDes" "RefDes" (pt 1050 mils 300 mils) (justify Left) (isVisible True) (textStyleRef "Default"))

	)
	(compDef "CX2016DB32000D0FLJCC" (originalName "CX2016DB32000D0FLJCC") (compHeader (numPins 4) (numParts 1) (refDesPrefix Y)
		)
		(compPin "1" (pinName "HOT_1") (partNum 1) (symPinNum 1) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "2" (pinName "GND_1") (partNum 1) (symPinNum 2) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "3" (pinName "HOT_2") (partNum 1) (symPinNum 3) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "4" (pinName "GND_2") (partNum 1) (symPinNum 4) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(attachedSymbol (partNum 1) (altType Normal) (symbolName "CX2016DB32000D0FLJCC"))
		(attachedPattern (patternNum 1) (patternName "CX2016DB32000D0FLJCC")
			(numPads 4)
			(padPinMap
				(padNum 1) (compPinRef "1")
				(padNum 2) (compPinRef "2")
				(padNum 3) (compPinRef "3")
				(padNum 4) (compPinRef "4")
			)
		)
		(attr "Manufacturer_Name" "AVX")
		(attr "Manufacturer_Part_Number" "CX2016DB32000D0FLJCC")
		(attr "Mouser Part Number" "581-CX2016DB32DFLJCC")
		(attr "Mouser Price/Stock" "https://www.mouser.com/Search/Refine.aspx?Keyword=581-CX2016DB32DFLJCC")
		(attr "Arrow Part Number" "CX2016DB32000D0FLJCC")
		(attr "Arrow Price/Stock" "https://www.arrow.com/en/products/cx2016db32000d0fljcc/avx")
		(attr "Description" "AVX - CX2016DB32000D0FLJCC - CRYSTAL, 32MHZ, 8PF, 2 X 1.6MM")
		(attr "Datasheet Link" "https://componentsearchengine.com/Datasheets/1/CX2016DB32000D0FLJCC.pdf")
		(attr "Height" "0.45 mm")
	)

)
