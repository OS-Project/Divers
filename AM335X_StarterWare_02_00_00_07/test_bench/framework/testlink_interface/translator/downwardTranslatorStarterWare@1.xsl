<?xml version="1.0"?>
<xsl:stylesheet xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
	version="1.0" >
	<xsl:output method="xml" indent="yes" encoding="utf-8"
		omit-xml-declaration="no" cdata-section-elements="id description name notes execute params"/>
	<xsl:param name="teeStafInstance"/>
	<xsl:variable name="CDATABegin" select="'&lt;![CDATA['" />
	<xsl:variable name="CDATAEnd" select="']]&gt;'" />
	<xsl:variable name="buildInfo"> 
		<xsl:value-of select="lower-case(top/build/notes)"/>
	</xsl:variable>
	<xsl:variable name="buildInfoSeq" select="tokenize($buildInfo,'(\s*[;=]+\s*)|(&lt;\s*/*.*?\s*&gt;)+')"/> 
	<xsl:variable name="platformIdx">
		<xsl:value-of select="index-of($buildInfoSeq,'platform')"/>
	</xsl:variable>
	<xsl:variable name="platformInfo">
		<xsl:value-of select="$buildInfoSeq[$platformIdx+1]"/>
	</xsl:variable>		
	<xsl:template match="/">
	<xsl:element name="test_session">
	<xsl:element name="testplan_id"/>
	<xsl:copy-of select="top/build"/>
	<xsl:copy-of select="top/testplan"/>  
	<xsl:for-each select="//testsuite/testcase | //testcases/testcase">
		<xsl:variable name="tcaseID"> 
			<xsl:value-of select="@internalid"/>
		</xsl:variable> 
		<xsl:variable name="tcaseextID"> 
			<xsl:value-of select="externalid"/>
		</xsl:variable> 
		<xsl:element name="testcase">
			<xsl:element name="id"><xsl:value-of select="$tcaseID"/>
			</xsl:element>
			<xsl:element name="extid"><xsl:value-of select="$tcaseextID"/>
			</xsl:element>
			<xsl:for-each select="custom_fields/custom_field">
			<xsl:variable name="elementName">
				<xsl:value-of select="name"/>
			</xsl:variable>
			<xsl:variable name="elementValue">
				<xsl:value-of select="value"/>
			</xsl:variable>
			<xsl:element name="{$elementName}">
			<xsl:value-of select="$CDATABegin" disable-output-escaping="yes" />
			<xsl:value-of select="$elementValue" disable-output-escaping="yes" />
			<xsl:value-of select="$CDATAEnd" disable-output-escaping="yes" />
			</xsl:element>
			</xsl:for-each>
			<xsl:element name="auto">1</xsl:element>
			<xsl:element name="platform">
				<xsl:value-of select="$platformInfo"/>
			</xsl:element>
			<xsl:element name="target"><xsl:value-of select="/top/build/name"/></xsl:element>
			<xsl:variable name="desc">
				<xsl:value-of select="summary"/>
			</xsl:variable>
			<xsl:element name="description"><xsl:value-of select="$desc"/></xsl:element>
		</xsl:element>
	</xsl:for-each>
	<xsl:element name="command">
	<xsl:element name="execute">process
	</xsl:element>
	<xsl:element name="params">START COMMAND perl  D:\/perlTools\/automation\/v6\/AutomationFramework\/TestLinkInterface\/parser\/StarterWare_TL_TEE_Link.pl
	</xsl:element>
	</xsl:element>
	</xsl:element>
	</xsl:template>
</xsl:stylesheet>
